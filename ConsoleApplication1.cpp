#include <stdio.h>
#include <gsl/gsl_odeiv2.h>
#include <gsl/gsl_errno.h>

// Система ОДУ: dy/dt = -100y
int func(double t, const double y[], double f[], void* params) {
    double lambda = *(double*)params;
    f[0] = -lambda * y[0];
    return GSL_SUCCESS;
}

int main() {
    // Коэффициент lambda
    double lambda = 100;

    // Создание системы ОДУ
    gsl_odeiv2_system sys = { func, NULL, 1, &lambda };

    // Выбор метода Рунге-Кутта 4 порядка
    gsl_odeiv2_step* s = gsl_odeiv2_step_alloc(gsl_odeiv2_step_rk4, 1);

    // Создание объекта для контроля точности
    gsl_odeiv2_control* c = gsl_odeiv2_control_y_new(1e-6, 0.0);

    // Создание объекта для эволюции системы
    gsl_odeiv2_evolve* e = gsl_odeiv2_evolve_alloc(1);

    // Начальные условия
    double t = 0.0, t1 = 1.0;
    double h = 1e-2;
    double y[1] = { 1.0 };

    // Интегрирование ОДУ
    while (t < t1) {
        int status = gsl_odeiv2_evolve_apply(e, c, s, &sys, &t, t1, &h, y);

        if (status != GSL_SUCCESS) {
            printf("Ошибка: %d\n", status);
            break;
        }

        printf("t = %.10e y = %.10e\n", t, y[0]);
    }

    // Освобождение ресурсов
    gsl_odeiv2_evolve_free(e);
    gsl_odeiv2_control_free(c);
    gsl_odeiv2_step_free(s);

    return 0;
}