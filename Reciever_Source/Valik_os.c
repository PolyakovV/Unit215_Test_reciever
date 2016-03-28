// Target : CC430F5137
#include <cc430x513x.h>
#include  <intrinsics.h>
#include "HAL/RF1A.h"
#include "HAL/cc430x613x_PMM.h"
#include "HAL/HAL_FLASH.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//#include "RF_Connection.h"
#include  "usart.h"
#include  "Valik_os.h"

unsigned long millis=0; 
int value_subscribers=0;
int value_processes=0;
int value_planned_tasks=0;

/**********************************************************************************/
/*      TIMER, interrupt service routine. Timer_A3 Interrupt Vector (TAIV) handler*/
/**********************************************************************************/
#pragma vector=TIMER1_A0_VECTOR

__interrupt void TIMER1_A0_ISR(void)
{
  
  TA1CCR0 +=16180; // 1ms period   
  millis++;
  
}

/****************************************************************************/
/*  Function name: delay_us                                                 */
/*  	Parameters                                                          */
/*          Input   :  p	                                            */
/*          Output  :  No	                                            */
/*	Action: Simple delay                                                */
/****************************************************************************/
void delay_1_us(volatile unsigned long p){}

void Delay_50us(void){
unsigned long l=43;//l=56;
  while (l){ l--;delay_1_us(1);}
}

///////////////////////////////////////////////////////////////////////
//
//
void Delay_ms(volatile unsigned long p){
unsigned long l=p*20;
while (l){ l--;Delay_50us();}
}
///////////////////////////////////////////////////////////////////////
//
//
int str_len (const char *str)
{
    return (*str) ? str_len(++str) + 1 : 0;
}

///////////////////////////////////////////////////////////////////////
//
//
int process_attach(int name, int prio, void *function)
{
        int i = 0;
        int ret = -1;
        while(i < MAX_PROCESSES) {
                if (name == 0) {
                        return ret;
                }
                if(processlist[i].attached != 1) {
                        processlist[i].pid = i;
                        processlist[i].name = name;
                        processlist[i].prio = prio;
                        processlist[i].function =(int *) function;
                        processlist[i].attached = 1;
                        value_processes++;
                        ret = 0;
                        break;
                }
                i++;
        }
        return ret;
 
}
///////////////////////////////////////////////////////////////////////
//
//
int process_detach(int pid)
{
        processlist[pid].attached = 0;
        return 0;
}


///////////////////////////////////////////////////////////////////////
// Detach process from process list
// int process_detach_by_name (int name)
// int name - predefined unique int
//
int process_detach_by_name (int name){

  for(int i = 0; i < MAX_PROCESSES; i++) {
                       
                        if(processlist[i].attached == 1) {
                          if(  name == processlist[i].name ) {
                            processlist[i].pid = 0;
                            processlist[i].prio =0;
                            processlist[i].attached = 0;
                            processlist[i].function =(int *) 0;
                            processlist[i].name = 0;

                          }
                        }
  }
  

  ////////compressing///////////

int i;  

int index_first_free_cell=0;
for (i=1;i<=MAX_PROCESSES;i++) {
  if(processlist[i-1].attached == 0){ // if cell empty
                                   if (!index_first_free_cell){index_first_free_cell = i;}
                                   }     
  if ((index_first_free_cell)&&(processlist[i-1].attached == 1)){
                              processlist[index_first_free_cell].pid = processlist[i].pid;
                              processlist[index_first_free_cell].prio = processlist[i].prio;
                              processlist[index_first_free_cell].attached = processlist[i].attached;
                              processlist[index_first_free_cell].function = (int *)processlist[i].function;
                              processlist[index_first_free_cell].name = processlist[i].name;
                              processlist[i].pid = 0;
                              processlist[i].prio =0;
                              processlist[i].attached = 0;
                              processlist[i].function = (int *)0;
                              processlist[i].name = 0;
                              i=index_first_free_cell;
                              index_first_free_cell=0;
                             }
}
value_processes=0;                    
while (processlist[value_processes].attached ==1){value_processes++;}

return 0;
}

///////////////////////////////////////////////////////////////////////
//  Core, main cycle, round 0
//  int scheduler() main cycle
//
int scheduler()
{
        int i = 0;
       void (*pgm_start_address)(void);
        while(1) {
                for(i = 0; i < value_processes; i++) {
                        if(processlist[i].attached == 1) {
                        pgm_start_address = (void(*)(void))processlist[i].function;
                        pgm_start_address();
                       // Delay_ms(200);Delay_ms(200);
                        }
                }
               
                for(i = 0; i < MAX_PLANNEDTASKS; i++) {
                  if ( planned_tasks_list[i].attached == 1 ) {
                          
                        if ( planned_tasks_list[i].time_to_start <= millis ) {    
                            process_trigger (planned_tasks_list[i].event) ;
                            if (planned_tasks_list[i].once_repeat == ONCE){
                                                                            clear_trigger_Events_by_Timer (planned_tasks_list[i].pid);
                                                                            } 
                                                                      else {
                                                                        if (planned_tasks_list[i].due_time <= millis) {
                                                                         clear_trigger_Events_by_Timer (planned_tasks_list[i].pid);
                                                                        }                                                                      
                                                                      }
                        }
                     
                  }
                }
        }
        return 0;
}

/////////////////////// Mediator ////////////////////////////
int process_subscribe (int subscr_event, char *subscr_name, int *function){
 
  int i = 0;
  int ret = -1;
                  if(str_len(subscr_name) > MAX_NAME_LEN) {
                                    return ret;
                            }
        while(i < MAX_SUBSCRIBERS) {
                 if (subscribers_list[i].attached != 1){
                        subscribers_list[i].pid = i;
                        subscribers_list[i].function =(int *) function;
                        strcpy(subscribers_list[i].subscr_name,subscr_name);
                        subscribers_list[i].subscr_event = subscr_event;
                        subscribers_list[i].attached = 1;
                        value_subscribers++;
                        ret = 0;
                        break;}
                i++;
        }
        return ret;
}

///////////////////////////////////////////////////////////////////////
//
//
int process_unsubscribe_by_name (char *subscr_name){
int i = 0;
  
while(i < MAX_SUBSCRIBERS){
    if (subscribers_list[i].attached == 1 ){
    if (strcmp(subscr_name,subscribers_list[i].subscr_name)==0){
                        subscribers_list[i].pid = 0;
                        subscribers_list[i].function = 0;
                        strcpy(subscribers_list[i].subscr_name,"");
                        subscribers_list[i].subscr_event = 0;
                        subscribers_list[i].attached = 0;
      }
    }
    i++;
    }
return 0;
}
///////////////////////////////////////////////////////////////////////
//
//
int process_unsubscribe_by_event (int subscr_event){
int i = 0;
  
while(i < MAX_SUBSCRIBERS){
    if (subscribers_list[i].attached == 1 ){
    if ( subscr_event == subscribers_list[i].subscr_event ){
                         subscribers_list[i].pid = 0;
                        subscribers_list[i].function = 0;
                        strcpy(subscribers_list[i].subscr_name,"");
                        subscribers_list[i].subscr_event = 0;
                        subscribers_list[i].attached = 0;
      }
    }
    i++;
    }
return 0;
}
//////////////////////////////////////////////////////////////////////
//
//
//
int process_trigger (int subscr_event){
  int i = 0;
  int (*pgm_start_address)(void);
  while(i < MAX_SUBSCRIBERS){
    if (subscribers_list[i].attached == 1 ){
    if ( subscr_event == subscribers_list[i].subscr_event ){
      pgm_start_address = (int(*)(void))subscribers_list[i].function; // get address of function
      //USART_Send_ROM_String("\n\r");
      //USART_Send_ROM_String(subscribers_list[i].subscr_name);
      pgm_start_address(); // go to this address
      }
    }
    i++;
    }
  return 0;
}

////////////////////////////////////////////////////////////////////////
//
//
//
int trigger_Event_On_Timer( int event, long time_to_start, long due_time,char once_repeat ){
   int i = 0;
        int ret = -1;
        while(i < MAX_PLANNEDTASKS) {
                if ( planned_tasks_list[i].attached != 1  ) {
                        planned_tasks_list[i].pid = i;
                        planned_tasks_list[i].event = event;
                        planned_tasks_list[i].time_to_start = millis + time_to_start;
                        planned_tasks_list[i].due_time = millis + time_to_start + due_time;
                        planned_tasks_list[i].once_repeat = once_repeat;
                        planned_tasks_list[i].attached = 1;
                        value_planned_tasks++;
                        ret = 0;
                        break;
                }
                i++;
        }
        return ret; 
 
}


int clear_trigger_Events_by_Timer (int pid){
int i = 0;
  
while(i < MAX_PLANNEDTASKS){
    if (planned_tasks_list[i].attached == 1 ){
      if (planned_tasks_list[i].pid == pid){
                        planned_tasks_list[i].pid = 0;
                        planned_tasks_list[i].event = 0;
                        planned_tasks_list[i].time_to_start = 0;
                        planned_tasks_list[i].due_time = 0;
                        planned_tasks_list[i].once_repeat = 0;
                        planned_tasks_list[i].attached = 0;
                        value_planned_tasks--; 
      }
    }
    i++;
    }
return 0;

}


long getMillis(){
return millis;
}




