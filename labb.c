#define _GNU_SOURCE
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <math.h>
#include <time.h>
#include "tick.h"
#include "BrickPi.h"
#include <sys/time.h>

#include <linux/i2c-dev.h>  
#include <fcntl.h>

//compile with "sudo gcc -o program "labb.c" -lrt -lm -lpthread"
//run with ./program

enum commandenum{STOP, FORWARD, BACKWARD, LEFT, RIGHT};

void *motor();
void *pressure();
void *distance();
void *randomLego();
void *straight();
void order_update(int u, int d, enum commandenum c, int s);
void timespec_add_us(struct timespec *t, long us);
int timespec_cmp(struct timespec *a, struct timespec *b);

struct order
{
	int urgent_level;
	int duration;
	enum commandenum command;
	int speed;

}order_status;

struct periodic_data {

  int index;
  long period_us;
  int wcet_sim;

};

int result;

pthread_mutex_t lock;

int main(){

	ClearTick();
	
	pthread_mutex_init(&lock, NULL);

	result = BrickPiSetup();
  	printf("BrickPiSetup: %d\n", result);
  	if(result)
    		return 0;

	BrickPi.Address[0] = 1;
  	BrickPi.Address[1] = 2;

	BrickPi.SensorType[PORT_1] = TYPE_SENSOR_ULTRASONIC_CONT;
	BrickPi.SensorType[PORT_2] = TYPE_SENSOR_TOUCH;
	BrickPi.MotorEnable[PORT_A] = 1;
  	BrickPi.MotorEnable[PORT_B] = 1;
	
	result = BrickPiSetupSensors();
  	printf("BrickPiSetupSensors: %d\n", result);
	if(result)
		return 0;

	pthread_t Motor, Pressure, Distance, RandomLego, Straight;
	pthread_attr_t my_attr;

	struct sched_param param;

	pthread_attr_init(&my_attr);
	pthread_attr_setschedpolicy(&my_attr, SCHED_FIFO);

	param.sched_priority = 1;
	pthread_attr_setschedparam(&my_attr, &param);
	pthread_create(&Motor, &my_attr, motor, NULL);

	param.sched_priority = 2;
	pthread_attr_setschedparam(&my_attr, &param);
	pthread_create(&Pressure, &my_attr, pressure, NULL);

	param.sched_priority = 3;
	pthread_attr_setschedparam(&my_attr, &param);
	pthread_create(&Distance, &my_attr, distance, NULL);

	param.sched_priority = 4;
	pthread_attr_setschedparam(&my_attr, &param);
	pthread_create(&RandomLego, &my_attr, randomLego, NULL);

	param.sched_priority = 5;
	pthread_attr_setschedparam(&my_attr, &param);
	pthread_create(&Straight, &my_attr, straight, NULL);

	
	while(1){
		result = BrickPiUpdateValues();
		//printf("%d", result);		
	}

	pthread_join(Motor, NULL);
	pthread_join(Pressure, NULL);	
	pthread_join(Distance, NULL);	
	pthread_join(RandomLego, NULL);
	pthread_join(Straight, NULL);
	
	return 0;
}

void *motor(){
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(0,&cpuset);
	pthread_setaffinity_np(pthread_self(),sizeof(cpu_set_t),&cpuset);
	
	struct timespec next, now;

	struct periodic_data interval; 
	interval.index = 0;
	interval.period_us = 100000;
	interval.wcet_sim = 0;

	while(1)
	{	
		clock_gettime(CLOCK_REALTIME, &next);
		timespec_add_us(&next, interval.period_us);
	
		//load();

		if(order_status.duration>0)
		{	
			printf("Thread: Motor \n");
			printf("%d \n", order_status.command);

			if(order_status.command==BACKWARD){
				BrickPi.MotorSpeed[PORT_A] = order_status.speed*-1;
		  		BrickPi.MotorSpeed[PORT_B] = order_status.speed*-1;
				printf("BACKWARD");
			}
			else if(order_status.command==FORWARD){
				BrickPi.MotorSpeed[PORT_A] = order_status.speed;
		  		BrickPi.MotorSpeed[PORT_B] = order_status.speed;
				printf("FORWARD");
			}
			else if(order_status.command==LEFT){
				BrickPi.MotorSpeed[PORT_A] = order_status.speed*-1;
		  		BrickPi.MotorSpeed[PORT_B] = order_status.speed*1;
				printf("LEFT");
			}
			else if(order_status.command==RIGHT){
				BrickPi.MotorSpeed[PORT_A] = order_status.speed*1;
		  		BrickPi.MotorSpeed[PORT_B] = order_status.speed*-1;
				printf("RIGHT");

			}else if(order_status.command==STOP){
				BrickPi.MotorSpeed[PORT_A] = 0;
		  		BrickPi.MotorSpeed[PORT_B] = 0;
				printf("STOP");
			}	
			

			order_status.duration--;
		}	
		else
		{
			BrickPi.MotorSpeed[PORT_A] = 0;
		  	BrickPi.MotorSpeed[PORT_B] = 0;
			order_status.urgent_level=0;
		}

		if (timespec_cmp(&now, &next) > 0)	{
			fprintf(stderr, "Deadline miss for thread %d\n", interval.index);
			fprintf(stderr, "now: %d sec %ld nsec next: %d sec %ldnsec \n",now.tv_sec, now.tv_nsec, next.tv_sec, next.tv_nsec);
		}


		clock_gettime(CLOCK_REALTIME, &now);
		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);	
	}
	

	pthread_exit(NULL);


}

void *pressure(){
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(0,&cpuset);
	pthread_setaffinity_np(pthread_self(),sizeof(cpu_set_t),&cpuset);

	struct timespec next;
	struct timespec now;

	struct periodic_data interval; 
	interval.index = 1;
	interval.period_us = 20000;
	interval.wcet_sim = 0;
		
	while(1)
	{	
			
		clock_gettime(CLOCK_REALTIME, &next);
		timespec_add_us(&next, interval.period_us);

		//load();
		
		if(BrickPi.SensorType[PORT_2]){
			printf("Thread: pressure \n");
			order_update(4, 1, STOP, 0);

		}

		if (timespec_cmp(&now, &next) > 0)	{
			fprintf(stderr, "Deadline miss for thread %d\n", interval.index);
			fprintf(stderr, "now: %d sec %ld nsec next: %d sec %ldnsec \n",now.tv_sec, now.tv_nsec, next.tv_sec, next.tv_nsec);
		}


		clock_gettime(CLOCK_REALTIME, &now);
		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
			
	}
	
	pthread_exit(NULL);

}

void *distance(){
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(0,&cpuset);
	pthread_setaffinity_np(pthread_self(),sizeof(cpu_set_t),&cpuset);

	struct timespec next;
	struct timespec now;

	struct periodic_data interval; 
	interval.index = 2;
	interval.period_us = 10000;
	interval.wcet_sim = 0;
	

	while(1)
	{	

		clock_gettime(CLOCK_REALTIME, &next);
		timespec_add_us(&next, interval.period_us);
		
		load();
		
		printf("Thread: Distance \n");

		int distance = BrickPi.Sensor[PORT_1];
		
		if(distance != 255 && distance != 127){
			if(distance != -1 && distance > 10){
				order_update(3, 1, FORWARD, 200);
			}else if(distance != -1 && distance <= 10){
				order_update(3, 1, BACKWARD, 200);
			}

		}

		if (timespec_cmp(&now, &next) > 0)	{
			fprintf(stderr, "Deadline miss for thread %d\n", interval.index);
			fprintf(stderr, "now: %d sec %ld nsec next: %d sec %ldnsec \n",now.tv_sec, now.tv_nsec, next.tv_sec, next.tv_nsec);
		}

		clock_gettime(CLOCK_REALTIME, &now);
		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
		
	}
	

	pthread_exit(NULL);


}

void *randomLego(){
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(0,&cpuset);
	pthread_setaffinity_np(pthread_self(),sizeof(cpu_set_t),&cpuset);
	
	srand(time(NULL));
	
	struct timespec next;
	struct timespec now;

	struct periodic_data interval; 
	interval.index = 3;
	interval.period_us = (rand()%100) * 30000;
	interval.wcet_sim = 0;

	int order = rand()%5;
	
	while(1)
	{
		clock_gettime(CLOCK_REALTIME, &next);
		timespec_add_us(&next, interval.period_us);

		//load();

		printf("Thread: RandomLego \n");
		order_update(1, 1, order, 200);
		order = rand()%5;

		if (timespec_cmp(&now, &next) > 0)	{
			fprintf(stderr, "Deadline miss for thread %d\n", interval.index);
			fprintf(stderr, "now: %d sec %ld nsec next: %d sec %ldnsec \n",now.tv_sec, now.tv_nsec, next.tv_sec, next.tv_nsec);
		}

		
		clock_gettime(CLOCK_REALTIME, &now);
		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
	}
	

	pthread_exit(NULL);


}

void *straight(){
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(0,&cpuset);
	pthread_setaffinity_np(pthread_self(),sizeof(cpu_set_t),&cpuset);

	struct timespec next;
	struct timespec now;

	struct periodic_data interval; 
	interval.index = 4;
	interval.period_us = 30000;
	interval.wcet_sim = 0;

	while(1)
	{	
	
		//load();
		clock_gettime(CLOCK_REALTIME, &next);
		timespec_add_us(&next, interval.period_us);

		printf("Thread: Straight \n");
		order_update(2, 1, FORWARD, 200);

		if (timespec_cmp(&now, &next) > 0)	{
			fprintf(stderr, "Deadline miss for thread %d\n", interval.index);
			fprintf(stderr, "now: %d sec %ld nsec next: %d sec %ldnsec \n",now.tv_sec, now.tv_nsec, next.tv_sec, next.tv_nsec);
		}


		clock_gettime(CLOCK_REALTIME, &now);
		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
	}
	
	pthread_exit(NULL);
}

void order_update(int u, int d, enum commandenum c, int s)
{
	pthread_mutex_lock(&lock);
	if(u>order_status.urgent_level)
	{
		order_status.urgent_level=u;
		order_status.duration=d;
		order_status.command=c;
		order_status.speed=s;
	}
	pthread_mutex_unlock(&lock);
}	

int load() {
	int i, num =1, primes = 0;
	while(num<=2000){
		i=2;
		while(i<=num){
			if(num%i==0)
				break;
			i++;
		}
		if(i==num)
			primes++;
		num++;
	}
	return primes;

}
void timespec_add_us(struct timespec *t, long us){
	t->tv_nsec += us*1000;
	if (t->tv_nsec > 1000000000) {
		t->tv_nsec = t->tv_nsec - 1000000000;
		t->tv_sec += 1;
	}
}

int timespec_cmp(struct timespec *a, struct timespec *b){
	if (a->tv_sec > b->tv_sec)
		return 1;
	else if (a->tv_sec < b->tv_sec)
		return -1;
	else if (a->tv_sec == b->tv_sec) {
		if (a->tv_nsec > b->tv_nsec)
			return 1;
		else if (a->tv_nsec == b->tv_nsec)
			return 0;
		else
			return -1;
	}
}
