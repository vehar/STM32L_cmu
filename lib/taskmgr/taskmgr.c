#include "taskmgr.h"


	
typedef struct
	{
	void *task[MAX_RUNNING_TASKS];
	volatile unsigned int running_tasks;
	}tasks_t;

volatile tasks_t tasks = {{0},0};


/*
0 - if task added successful
1 - if MAX_RUNNING_TASKS exceed
*/

uint8_t get_runningtasks(void)
{
	return tasks.running_tasks;
}


uint8_t task_add(void *task)
{
    if(tasks.running_tasks>=MAX_RUNNING_TASKS) return 1;
    tasks.task[tasks.running_tasks] = task;
    tasks.running_tasks++;
    return 0;
}

/*
0 - if process found and killed
1 - if process not found
*/

uint8_t task_kill(uint8_t pid)
{
    if(pid>tasks.running_tasks) return 1;

    for(;pid!=tasks.running_tasks-1;pid++)
    {
        tasks.task[pid]=tasks.task[pid+1];
    }
    tasks.running_tasks--;
    return 0;
}

void taskmgr(void)
{
    while(tasks.running_tasks)
    {
        uint8_t pid=0;
        for(pid = 0;pid<tasks.running_tasks;pid++)
        {
           void (*task)(uint8_t pid) = tasks.task[pid];
           task(pid);
        }
    }
}
