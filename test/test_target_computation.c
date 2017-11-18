#include <stdio.h>
#include <stdlib.h>


#define SIGN(x) (x > 0 ? 1 : (-1))

float get_position(float, float, float, float, float);

/*
prend en argument :
    le temps t
    les caractéristiques de la courbe de vitesse, à savoir :
        a_montante : accélération lors de la phase d'augmentation de vitesse
        a_descendante : accélération lors de la phase de freinage
        v_croisiere : vitesse de croisière
        x_final : la destination du robot

    NB : a_montante et a_descendante doivent être de signe opposé
    (a_montante peut être négative dans le cas où le robot recule)

renvoie la position à l'instant t

pour memoire, la courbe de vitesse ressemble à :

         ---------------------
        /                     \
       /                       \
------/                         \--------

(ou (-1) *  ça )

car c'est la courbe qui permet de minimiser le temps pour atteindre x_final
tout en ayant une vitesse continue (en accord avec la physique)

*/
float get_position(float t, float a_montante, float a_descendante,
                   float v_croisiere, float x_final)
{


    float t1 = v_croisiere / a_montante;
    float t2 = x_final / v_croisiere + v_croisiere / 2 * (1 / a_montante + 1 / a_descendante);

    //printf("%.3f \t %.3f\n", t1, t2);


    //cas où on atteint jamais la vitesse de croisière
    if (t2 <= t1){

        float t4_carre = 2 * x_final / (a_montante * (1 - a_montante / a_descendante));

        if (t * t <= t4_carre) return a_montante * t * t / 2;

        //calcul (demoniaque) de 2 / sqrt(t4_carre)
        //noter qu'il s'agit quand même d'une valeur approchée...
        //mais avec 3 décimales exactes
        int i = *(int*)& t4_carre;
        i = 0x5f3759df - (i >> 1);
        float t4_inv = *(float*)&i;
        t4_inv = t4_inv * (3.0f - (t4_carre * t4_inv * t4_inv));

        float delta_t = t - t4_inv * x_final / a_montante;

        if (delta_t >= 0) return x_final;

        return a_descendante / 2 * delta_t * delta_t + x_final;

    }

    float t3 = t2 - v_croisiere /a_descendante;


    if (t < 0) return 0;
    else if (t <= t1 && t <= t2) return a_montante * t * t / 2;
    else if (t <= t2) return t1 * v_croisiere / 2 + v_croisiere * (t - t1);
    else if (t <= t3)
        return x_final + v_croisiere * v_croisiere / (2 * SIGN(x_final) * a_descendante) + (t - t2) * (v_croisiere + SIGN(x_final) * a_descendante * (t - t2) / 2);
        //return x_final + v_croisiere * v_croisiere / 2 / a_descendante \
        //    + (t - t2) * (v_croisiere + a_descendante / 2 * (t - t2));
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

    /*for (i = 0; i < N; i++){

        printf("x(%.2f) = %.4f\n", ((float) i) / N * t_max, get_position(((float) i) / N * t_max, 1., -2., 3, 15));
    }*/

    for (i = 0; i < N; i++)printf("%f, ", ((float) i) / N * t_max);
    puts("");
    for (i = 0; i < N; i++){
        //get_position(((float) i) / N * t_max, a_m, a_d, v_c, x_f);
        printf("%f, ", get_position(((float) i) / N * t_max, a_m, a_d, v_c, x_f));
    }

    puts("");

    return 0;
}
