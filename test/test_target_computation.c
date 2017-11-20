#include <stdio.h>
#include <stdlib.h>

#include <math.h>


#define SIGN(x) (x > 0 ? 1 : (-1))
#define ABS(x) (x > 0 ? x : -x)

float get_position(float, float, float, float, float);

/*
prend en argument :
    le temps t
    les caract�ristiques de la courbe de vitesse, � savoir :
        a_montante : acc�l�ration lors de la phase d'augmentation de vitesse
        a_descendante : acc�l�ration lors de la phase de freinage
        v_croisiere : vitesse de croisi�re
        x_final : la destination du robot

    NB : a_montante et a_descendante doivent �tre de signe oppos�
    (a_montante peut �tre n�gative dans le cas o� le robot recule)

renvoie la position � l'instant t

pour memoire, la courbe de vitesse ressemble � :

         ---------------------
        /                     \
       /                       \
------/                         \--------

(ou (-1) *  �a )

car c'est la courbe qui permet de minimiser le temps pour atteindre x_final
tout en ayant une vitesse continue (en accord avec la physique)

*/
float get_position(float t, float a_montante, float a_descendante,
                   float v_croisiere, float x_final)
{

    if (x_final >= 0) {
      a_montante = ABS(a_montante);
      a_descendante = -ABS(a_descendante);
      v_croisiere = ABS(v_croisiere);
    }
    else {
      a_montante = - ABS(a_montante);
      a_descendante = ABS(a_descendante);
      v_croisiere = - ABS(v_croisiere);
    }


    float t1 = v_croisiere / a_montante;
    float t2 = x_final / v_croisiere + v_croisiere / 2 * (1 / a_montante + 1 / a_descendante);

    //cas o� on atteint jamais la vitesse de croisi�re
    if (t2 <= t1){
        float t4_carre = 2 * x_final / (a_montante * (1 - a_montante / a_descendante));

        if (t * t <= t4_carre) return a_montante * t * t / 2;

        float t4_inv = 2 / sqrt(t4_carre);
        float delta_t = t - t4_inv * x_final / a_montante;

        if (delta_t >= 0) return x_final;
        return a_descendante / 2 * delta_t * delta_t + x_final;
    }

    float t3 = t2 - v_croisiere /a_descendante;

    if (t < 0) return 0;
    else if (t <= t1 && t <= t2) return a_montante * t * t / 2;
    else if (t <= t2) return t1 * v_croisiere / 2 + v_croisiere * (t - t1);
    else if (t <= t3)
      return x_final + v_croisiere * v_croisiere / 2 / a_descendante \
              + (t - t2) * (v_croisiere + a_descendante / 2 * (t - t2));
    else return x_final;
}

int main(int argc, char* argv [])
{

    int i;
    int N = 1000;
    float t_max = 10;

    if (argc != 5) {
        puts("ERREUR : mauvais usage de ce programme");
        puts("les arguments du programmes doivent etre : a_montante \
             a_descendante v_croisiere x_final");
    }
    float a_m = strtof(argv[1], NULL);
    float a_d = strtof(argv[2], NULL);
    float v_c = strtof(argv[3], NULL);
    float x_f = strtof(argv[4], NULL);

    for (i = 0; i < N; i++)printf("%f, ", ((float) i) / N * t_max);
    puts("");
    for (i = 0; i < N; i++){
        printf("%f, ", get_position(((float) i) / N * t_max, a_m, a_d, v_c, x_f));
    }

    puts("");
    return 0;
}
