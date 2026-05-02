
#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"

extern int crossroads_step;
void* unitstep_changed();

// 단위 스텝 barrier
static struct lock      step_lock;
static struct condition step_cond;
static int active_vis;   // 아직 목적지에 도달하지 않은 차량 수 
static int arrived_vis;  // 이번 스텝에서 barrier에 도달한 차량 수
static int leaving_vis;  // 이번 스텝에서 종료 예정인 차량 수    
static int barrier_step; // 의도치 않은 스텝 증가로 인한 wakeup 방지

// 교차로 진입 제어 
static struct lock intersection_lock;
static int intersection_count; // 현재 교차로 내부 차량 수
#define MAX_INTERSECTION 3     // 최대 동시 진입 가능 대수 
#define DEBUG 1

/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][10] = {
	/* from A */ {
		/* to A */
		{{-1,-1},},
		/* to B */
		{{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}}
	},
	/* from B */ {
		/* to A */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
		/* to B */
		{{-1,-1},},
		/* to C */
		{{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from C */ {
		/* to A */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
		/* to C */
		{{-1,-1},},
		/* to D */
		{{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from D */ {
		/* to A */
		{{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
		/* to D */
		{{-1,-1},}
	}
};

static int is_position_outside(struct position pos)
{
	return (pos.row == -1 || pos.col == -1);
}

// 교차로 내부 셀인지 아닌지(row, col 모두 [2, 3, 4] 범위)
static int is_in_intersection(struct position pos)
{
	return pos.row >= 2 && pos.row <= 4 &&
	       pos.col >= 2 && pos.col <= 4;
}

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
	struct position pos_cur, pos_next;

	pos_next = vehicle_path[start][dest][step];
	pos_cur  = vi->position;

	if (vi->state == VEHICLE_STATUS_RUNNING) {
		// 다음 위치가 맵 밖이면 현재 칸을 해제하고 종료 
		if (is_position_outside(pos_next)) {
			vi->position.row = vi->position.col = -1;
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			return 0;
		}
	}

	// 교차로 진입/퇴장 여부
	int entering = !is_in_intersection(pos_cur) && is_in_intersection(pos_next);
	int leaving  =  is_in_intersection(pos_cur) && !is_in_intersection(pos_next);

	// 교차로 진입 시 카운터 증가
	if (entering) {
		lock_acquire(&intersection_lock);
		#if DEBUG 
		printf("Vehicle %c trying to enter intersection (current count: %d)\n", vi->id, intersection_count);
		#endif
		if (intersection_count >= MAX_INTERSECTION) {
			lock_release(&intersection_lock);
			return -1; // 진입 실패(현재 위치 유지)
		}
		intersection_count++;
		lock_release(&intersection_lock);
	}

	// 다음 칸 획득 실패 시, 현재 위치 유지
	if (!lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])) {
		// 진입 실패 시 카운터 감소 후 현재 칸 해제
		if (entering) {
			lock_acquire(&intersection_lock);
			intersection_count--;
			lock_release(&intersection_lock);
		}
		return -1;
	}

	// 진입 성공 시 상태 변경
	if (vi->state == VEHICLE_STATUS_READY) {
		vi->state = VEHICLE_STATUS_RUNNING;
	} else {
		// 교차로 퇴장 시 카운터 감소
		if (leaving) {
			#if DEBUG
			printf("Vehicle %c leaving intersection (current count: %d)\n", vi->id, intersection_count);
			#endif
			lock_acquire(&intersection_lock);
			intersection_count--;
			lock_release(&intersection_lock);
		}
		// 현재 칸 락 해제 
		lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
	}

	// 다음 칸으로 이동
	vi->position = pos_next;
	return 1;
}

void init_on_mainthread(int thread_cnt)
{
	active_vis  = thread_cnt;
	arrived_vis = 0;
	leaving_vis = 0;
	barrier_step       = 0;

	lock_init(&step_lock);
	cond_init(&step_cond);

	lock_init(&intersection_lock);
	intersection_count = 0;
}

// 메인 루프
void vehicle_loop(void *_vi)
{
	int res;
	int start, dest;
	int local_step = 0; // 각 차량 별 경로상 현재 위치
	bool will_leave = false;

	struct vehicle_info *vi = _vi;

	start = vi->start - 'A'; // A:0 B:1 C:2 D:3
	dest  = vi->dest  - 'A';

	vi->position.row = vi->position.col = -1;
	vi->state = VEHICLE_STATUS_READY;

	while (1) {
		res = try_move(start, dest, local_step, vi); // -1, 0, 1
		if (res == 1) // 이동 성공
			local_step++;
		if (res == 0) // 다음 위치가 맵 밖일 때(목적지)
			will_leave = true;

		// 단위 스텝 락
		lock_acquire(&step_lock);
		int my_step = barrier_step; 
		arrived_vis++;
		if (will_leave)
			leaving_vis++;

		if (arrived_vis == active_vis) {
			// 모든 차량이 도착하면 단위 스텝 증가
			crossroads_step++;
			unitstep_changed(); 

			active_vis  -= leaving_vis; // 도착한 차량 수만큼 active_vis 감소
			arrived_vis  = 0;
			leaving_vis  = 0;
			barrier_step++; // 모든 차량 도착 후 barrier_step 증가 (의도치 않은 스텝 증가로 인한 wakeup 방지)
			cond_broadcast(&step_cond, &step_lock);
		} else {
			// 아직 도착하지 않은 차량이 있으면 락을 풀고 대기
			while (my_step == barrier_step)
				cond_wait(&step_cond, &step_lock);
		}
		lock_release(&step_lock);

		if (will_leave)
			break;
	}

	vi->state = VEHICLE_STATUS_FINISHED;
}
