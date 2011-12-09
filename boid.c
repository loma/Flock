#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <cv.h>
#include <highgui.h> 

#define BOIDS_NUM   100
#define WIDTH       300
#define HEIGHT      300

float   r           = 2.0f;     // radius of a boid
float   maxforce    = 0.05f;     // max force
float   maxspeed    = 2.0f;    // max speed

typedef struct {
    float x;
    float y;
} vector;

typedef struct {
    vector loc;
    vector vel;
    vector acc;
} Boid;

// add vector a to b, and keep result to a
void add_vector (vector *a, vector *b) {
	a->x += b->x;
	a->y += b->y;
}
// subtract vector a by b and keep result to a
void sub_vector (vector *a, vector *b) {
	a->x -= b->x;
	a->y -= b->y;
}
// mul vector by a scalar
void mul_vector (vector *a, float b) {
	a->x *= b;
	a->y *= b;
}
// div vector by a scalar
void div_vector (vector *a, float b) {
	a->x /= b;
	a->y /= b;
}
// distance of a 2 vectors
float dis_vector (vector a, vector b) {
	int x = a.x - b.x;
	int y = a.y - b.y;
	
	return (float) sqrt((x*x) + (y*y));
}
// magnitude of a vector
float mag_vector (vector *a) {
    int x = a->x;
    int y = a->y;
    return sqrt(x*x + y*y);
}
// normalise vector
void norm_vector (vector *a) {
	float dist  = mag_vector(a);
	div_vector(a, dist);
}
// limit the vector
void limit (vector *a) {
    if (mag_vector(a) > maxforce) {
        norm_vector(a);
        mul_vector(a, maxforce);
    }
}

vector seperate (int id, Boid *b) {
    float desiredseparation = 25.0f;
    vector sum;
    sum.x           = 0;
    sum.y           = 0;
    
    int count       = 0;
    int i;
    float dist      = 0;
    for (i=0; i<BOIDS_NUM; i++) {
        dist = dis_vector(b[i].loc, b[id].loc);
        if (dist > 0 && dist < desiredseparation) {
            vector diff;
            diff.x      = b[id].loc.x;
            diff.y      = b[id].loc.y;
            
            sub_vector(&diff, &b[i].loc);
            norm_vector(&diff);
            div_vector(&diff, dist);
            add_vector(&sum, &diff);
            count++;
        }
    }
    
    if (count > 0) {
        div_vector(&sum, count);
    }
    
    return sum;
}

vector align (int id, Boid *b) {
    float neighbordist = 50.0f;
    vector sum;
    sum.x           = 0;
    sum.y           = 0;
    int count       = 0;
    int i;
    float dist;
    for (i=0; i<BOIDS_NUM; i++) {
        dist = dis_vector(b[i].loc, b[id].loc);
        if (dist > 0 && dist < neighbordist) {
            add_vector(&sum, &b[i].vel);
            count++;
        }
    }
    
    if (count > 0) {
        div_vector(&sum, count);
        limit(&sum);
    }
    
    return sum;
}    

vector cohesion (int id, Boid *b) {
    float neighbordist = 50.0f;  
    vector sum;
    sum.x           = 0;
    sum.y           = 0;
    int count       = 0;
    int i;
    float dist;
    for (i=0; i<BOIDS_NUM; i++) {
        dist = dis_vector(b[i].loc, b[id].loc);
        if (dist > 0 && dist < neighbordist) {
            add_vector(&sum, &b[i].loc);
            count++;
        }
    }
    
    if (count > 0) {
        div_vector(&sum, count);
        // steer the boid
        vector steer;
        steer.x     = 0;
        steer.y     = 0;
        
        sub_vector(&sum, &b[id].loc);
        dist        = mag_vector(&sum);
        
        if (dist > 0) {
            norm_vector(&sum);
            mul_vector(&sum, maxspeed);
            steer   = sum;
            sub_vector(&steer, &b[id].vel);
            limit(&steer);
        }
        
        return steer;
    }
    
    return sum;
}

vector wall (int id, Boid *b) {
    float desiredseparation = 15.0f;
    vector sum;
    sum.x           = 0;
    sum.y           = 0;
    
    int count       = 0;
    int i;
    float dist      = 0;
    vector temp;
    
    for (i=0; i<WIDTH; i++) {
        temp.y      = 0;
        temp.x      = i;
        dist = dis_vector(temp, b[id].loc);
        if (dist > 0 && dist < desiredseparation) {
            vector diff;
            diff.x      = b[id].loc.x;
            diff.y      = b[id].loc.y;
            
            sub_vector(&diff, &temp);
            norm_vector(&diff);
            div_vector(&diff, dist);
            add_vector(&sum, &diff);
            count++;
        }
        
        temp.y      = HEIGHT-1;
        temp.x      = i;
        dist = dis_vector(temp, b[id].loc);
        if (dist > 0 && dist < desiredseparation) {
            vector diff;
            diff.x      = b[id].loc.x;
            diff.y      = b[id].loc.y;
            
            sub_vector(&diff, &temp);
            norm_vector(&diff);
            div_vector(&diff, dist);
            add_vector(&sum, &diff);
            count++;
        }
        
    }
    
    for (i=0; i<HEIGHT; i++) {
        temp.y      = i;
        temp.x      = 0;
        dist = dis_vector(temp, b[id].loc);
        if (dist > 0 && dist < desiredseparation) {
            vector diff;
            diff.x      = b[id].loc.x;
            diff.y      = b[id].loc.y;
            
            sub_vector(&diff, &temp);
            norm_vector(&diff);
            div_vector(&diff, dist);
            add_vector(&sum, &diff);
            count++;
        }
        
        temp.y      = i;
        temp.x      = WIDTH-1;
        dist = dis_vector(temp, b[id].loc);
        if (dist > 0 && dist < desiredseparation) {
            vector diff;
            diff.x      = b[id].loc.x;
            diff.y      = b[id].loc.y;
            
            sub_vector(&diff, &temp);
            norm_vector(&diff);
            div_vector(&diff, dist);
            add_vector(&sum, &diff);
            count++;
        }
        
    }
    
    if (count > 0) {
        div_vector(&sum, count);
    }
    
    return sum;
}

// calculate acceleration
vector flock (int id, Boid *b) {
    vector sep      = seperate(id, b);
    vector ali      = align(id, b);
    vector coh      = cohesion(id, b);
    vector w        = wall(id, b);
    
    vector acc;
    acc.x           = 0;
    acc.y           = 0;
    
    mul_vector(&sep, 3.0f);
    mul_vector(&ali, 1.0f);
    mul_vector(&coh, 1.0f);
    mul_vector(&w, 5.0f);
    
    add_vector(&acc, &sep);
    add_vector(&acc, &ali);
    add_vector(&acc, &coh);
    add_vector(&acc, &w);
    
    return acc;
}

void border (vector *a) {
    if (a->x < 0) a->x += WIDTH;
    if (a->y < 0) a->y += HEIGHT;
    if (a->x >= WIDTH) a->x -= WIDTH;
    if (a->y >= HEIGHT) a->y -= HEIGHT;
}

// update location of a boid
void update (int id, Boid *b, Boid *b2) {
    vector acc      = flock(id, b);
    
    b2[id].vel = b[id].vel;
    
    add_vector(&b2[id].vel, &acc);
    
    limit(&b2[id].vel);
    
    add_vector(&b2[id].loc, &b2[id].vel);
    
    border (&b2[id].loc);
}

int main () {
    
    Boid *b     = (Boid*) malloc (sizeof(Boid) * BOIDS_NUM);
    Boid *b2    = (Boid*) malloc (sizeof(Boid) * BOIDS_NUM);
    int i;
    for (i=0; i<BOIDS_NUM; i++) {
        b[i].loc.x      = WIDTH/2;
        b[i].loc.y      = HEIGHT/2;
        
        b[i].vel.x      = ((rand()%100000)/50000.0f) - 1.0f;
        b[i].vel.y      = ((rand()%100000)/50000.0f) - 1.0f;

    }
    
    IplImage* g = cvCreateImage(cvSize(WIDTH, HEIGHT),IPL_DEPTH_8U,3);
    
    int frame;
    for (frame=0; frame<10000; frame++) {
        
        memset(g->imageData, 255, g->imageSize);        
        for (i=0; i<BOIDS_NUM; i++) {
            cvCircle(g, cvPoint(b[i].loc.x, b[i].loc.y), 2, cvScalar(0,0,0,0), -1, 8, 0);
		} 
               
        for (i=0; i<BOIDS_NUM; i++) {
            update(i, b, b2);
        }
        
        memcpy(b, b2, sizeof(Boid) * BOIDS_NUM);

        
        cvShowImage("g", g);
        cvWaitKey(1);
    }
    
    
    return 0;
}
