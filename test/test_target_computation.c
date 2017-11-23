#include <stdio.h>
#include <stdlib.h>

#include <math.h>


#define SIGN(x) (x > 0 ? 1 : (-1))
#define ABS(x) (x > 0 ? x : -x)

float get_target(float, float, float, float, float);

/*
parameters :
  t                 : time, in seconds. t = 0 is the begginning of the move
                      must be positive
  inc_acceleration  : acceleration when the speed of the robot goes increases
                      in m / s^2
  dec_acceleration  : acceleration when the speed of the robot goes decreases
                      in m / s^2
  cruising_speed    : maximum speed of the robot
                      in m / s
  final_x           : final destination of the robot, in m

  NOTE :
    only the signs of t and final_x matter
    the appropriate signs for inc_acceleration, dec_acceleration, cruising_speed
    are calculated

returns  the intermediate target to reach at time t

speed graph :
          ----------------------
        / ^                     ^\
       /  |                     | \
------/   |                     |  \--------
      ^   ^                     ^  ^
      |   |                     |  |
      |   |                     |  |
    t=0  t=t1                 t=t3 t=t4

*/
float get_target(float t, float inc_acceleration, float dec_acceleration,
                   float cruising_speed, float final_x)
{

    /*  signs computation */
    if (final_x >= 0) {
      inc_acceleration  =   ABS(inc_acceleration);
      dec_acceleration  = - ABS(dec_acceleration);
      cruising_speed    =   ABS(cruising_speed);
    }
    else {
      inc_acceleration  = - ABS(inc_acceleration);
      dec_acceleration  =   ABS(dec_acceleration);
      cruising_speed    = - ABS(cruising_speed);
    }

    /* t1 and t2 computation, see graph above */
    float t1 = cruising_speed / inc_acceleration;
    float t2 = final_x / cruising_speed + cruising_speed / 2 * (1 / inc_acceleration + 1 / dec_acceleration);

    /* if cruising_speed is never reached */
    if (t2 <= t1){
        float t4_square = 2 * final_x / (inc_acceleration * (1 - inc_acceleration / dec_acceleration));

        if (t * t <= t4_square) return inc_acceleration * t * t / 2;

        float t4_inv = 2 / sqrt(t4_square);
        float delta_t = t - t4_inv * final_x / inc_acceleration;

        if (delta_t >= 0) return final_x;
        return dec_acceleration / 2 * delta_t * delta_t + final_x;
    }

    float t3 = t2 - cruising_speed /dec_acceleration;

    /* the result is computed in function of the part of the graph at which t belongs */
    if (t < 0) return 0;
    else if (t <= t1 && t <= t2) return inc_acceleration * t * t / 2;
    else if (t <= t2) return t1 * cruising_speed / 2 + cruising_speed * (t - t1);
    else if (t <= t3)
      return final_x + cruising_speed * cruising_speed / 2 / dec_acceleration \
              + (t - t2) * (cruising_speed + dec_acceleration / 2 * (t - t2));
    else return final_x;
}

int main(int argc, char* argv [])
{

    int i;
    int N = 1000;
    float t_max = 10;

    if (argc != 5) {
        puts("ERREUR : mauvais usage de ce programme");
        puts("les arguments du programmes doivent etre : inc_acceleration \
             dec_acceleration cruising_speed final_x");
    }
    float a_m = strtof(argv[1], NULL);
    float a_d = strtof(argv[2], NULL);
    float v_c = strtof(argv[3], NULL);
    float x_f = strtof(argv[4], NULL);

    for (i = 0; i < N; i++)printf("%f, ", ((float) i) / N * t_max);
    puts("");
    for (i = 0; i < N; i++){
        printf("%f, ", get_target(((float) i) / N * t_max, a_m, a_d, v_c, x_f));
    }

    puts("");
    return 0;
}
