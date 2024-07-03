#include <stdio.h>
#include <gsl/gsl_odeiv2.h>
#include <gsl/gsl_errno.h>

// Функция системы ОДУ
int func(double t, const double y[], double f[], void* params) {
    double lambda = *(double*)params;
    f[0] = lambda * y[0];
    return GSL_SUCCESS;
}

// Выполнение шагов интегрирования и запись результатов
void perform_steps_and_write_results(const gsl_odeiv2_step_type* T, double h, double y[], double t, double tk, double lambda, const char* filename) {
    gsl_odeiv2_step* s = gsl_odeiv2_step_alloc(T, 1);
    gsl_odeiv2_control* c = gsl_odeiv2_control_y_new(1e-8, 0.0);
    gsl_odeiv2_evolve* e = gsl_odeiv2_evolve_alloc(1);

    gsl_odeiv2_system sys = { func, NULL, 1, &lambda };

    FILE* file;
    errno_t err;

    err = fopen_s(&file, filename, "w");
    if (err != 0) {
        fprintf(stderr, "Не удалось открыть файл.\n");
        return;
    }

    double h_step = h;
    while (t < tk) {
        int status = gsl_odeiv2_evolve_apply(e, c, s, &sys, &t, tk, &h_step, y);

        if (status != GSL_SUCCESS) {
            printf("Ошибка: %s\n", gsl_strerror(status));
            break;
        }

        // Запись результатов на каждом шаге
        fprintf(file, "%.10f %.10f %.10f\n", t, y[0], h_step);
    }

    fclose(file);
    gsl_odeiv2_evolve_free(e);
    gsl_odeiv2_control_free(c);
    gsl_odeiv2_step_free(s);
}

int main() {
    double lambda = -100.0;
    double h0 = 1e-2;
    double tk = 1.0;
    double y[1] = { 1.0 }; // Начальное условие y(0) = 1

    // Решение ОДУ различными методами и запись результатов
    perform_steps_and_write_results(gsl_odeiv2_step_rk4, h0, y, 0.0, tk, lambda, "rk4_results.csv");
    y[0] = 1.0; // Сброс начального условия
    perform_steps_and_write_results(gsl_odeiv2_step_rk8pd, h0, y, 0.0, tk, lambda, "rk8pd_results.csv");
    y[0] = 1.0; // Сброс начального условия
    perform_steps_and_write_results(gsl_odeiv2_step_rk2, h0, y, 0.0, tk, lambda, "rk2_results.csv");

    return 0;
}
