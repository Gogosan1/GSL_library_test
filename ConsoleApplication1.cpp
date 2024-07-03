#include <stdio.h>
#include <gsl/gsl_odeiv2.h>
#include <gsl/gsl_errno.h>

// Функция системы ОДУ
int func(double t, const double y[], double f[], void* params) {
    double* lambda = (double*)params;
    f[0] = lambda[0] * y[0]; // dy1/dt = y1
    f[1] = lambda[1] * y[1]; // dy2/dt = -100y2
    return GSL_SUCCESS;
}

// Главная функция
int main() {
    double lambda[2] = { 1.0, -100.0 }; // Параметры системы: λ1 = 1, λ2 = -100
    double y[2] = { 1.0, 1.0 };         // Начальные условия: y1(0) = 1, y2(0) = 1
    double t = 0.0;                   // Начальное время
    double t1 = 1.0;                  // Конечное время: tk = 1
    double h = 1e-2;                  // Начальный шаг: h0 = 10^(-2)

    gsl_odeiv2_step* s = gsl_odeiv2_step_alloc(gsl_odeiv2_step_rk4, 2);
    gsl_odeiv2_control* c = gsl_odeiv2_control_y_new(1e-10, 0.0);
    gsl_odeiv2_evolve* e = gsl_odeiv2_evolve_alloc(2);
    gsl_odeiv2_system sys = { func, NULL, 2, lambda };

    FILE* file_y1;
    errno_t err;

    // Открытие файлов для записи результатов
    err = fopen_s(&file_y1, "y1_results.csv", "w");
    if (err != 0) {
        fprintf(stderr, "Не удалось открыть файл y1_results.csv.\n");
        return -1;
    }

    // Процесс интегрирования
    while (t < t1) {
        int status = gsl_odeiv2_evolve_apply(e, c, s, &sys, &t, t1, &h, y);
        if (status != GSL_SUCCESS) {
            fprintf(stderr, "Ошибка при интегрировании: %d\n", status);
            break;
        }
        // Запись результатов с точностью до 10 знаков после запятой
        fprintf(file_y1, "%.10f %.10f %.10f %.10f\n", t, y[0], y[1], h);
    }

    // Закрытие файлов
    fclose(file_y1);

    // Освобождение ресурсов
    gsl_odeiv2_evolve_free(e);
    gsl_odeiv2_control_free(c);
    gsl_odeiv2_step_free(s);

    return 0;
}
