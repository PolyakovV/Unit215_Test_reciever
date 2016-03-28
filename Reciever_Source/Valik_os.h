#define MAX_PROCESSES 6
#define MAX_SUBSCRIBERS  30
#define MAX_PLANNEDTASKS 30

#define MAX_NAME_LEN 20

//////Planned tasks predefined////

#define ONCE       0
#define REPEAT     1



struct pcb{
        int pid;       /*  process ID*/
        int prio;      /* priority of process */
        int attached;  /* 1 if attached, if no - 0 */
        int *function; /* function address  */
        int name;      /* process name defined*/
};

static  pcb processlist[MAX_PROCESSES]; // create list

struct subscriber_str{
                      int pid; /*  process ID*/
                      int *function; /* function address  */
                      char  subscr_name[MAX_NAME_LEN]; /* process name */
                      int subscr_event; /* defined name of trigger event int*/
                      int attached; /* 1 if attached, if no - 0 */
};


static  subscriber_str subscribers_list[MAX_SUBSCRIBERS]; // create list


struct planned_task_str{
                      int pid; /*  process ID*/
                      int event; /* function address  */
                      long time_to_start;
                      long due_time;
                      char once_repeat;
                      int attached; /* 1 if attached, if no - 0 */
                      
};


static  planned_task_str planned_tasks_list[MAX_PLANNEDTASKS]; // create list


extern long getMillis();
///////////External Function//////////////

extern int process_attach(int name, int prio, void *function);
extern int process_detach(int pid);
extern int process_detach_by_name (int name);
extern int scheduler();

/////////////Mediator///////////////////////
extern int process_subscribe (int subscr_event, char *subscr_name, int *function);
extern int process_unsubscribe_by_name (char *subscr_name);
extern int process_unsubscribe_by_event (int subscr_event);
extern int process_trigger (int subscr_event);

////////////Timer////////////////////////////
int clear_trigger_Events_by_Timer (int pid);
int trigger_Event_On_Timer( int event, long time_to_start, long due_time, char once_repeat );