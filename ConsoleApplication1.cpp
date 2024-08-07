﻿#include <stdio.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

// Система дифференциальных уравнений
int func(double t, const double y[], double f[], void* params) {
    double lambda2 = *(double*)params;
    double lambda3 = *((double*)params + 1);

    f[0] = -y[0] + 1e8 * y[2] * (1 - y[0]);
    f[1] = -10 * y[1] + 3e7 * y[2] * (1 - y[1]);
    f[2] = -f[0] - f[1];

    return GSL_SUCCESS;
}

int main() {
    // Параметры lambda2 и lambda3
    double lambda_params[] = { -1, -3e7 }; // Начальные значения, измените их по вашим требованиям

    gsl_odeiv2_system sys = { func, NULL, 3, lambda_params };

    gsl_odeiv2_driver* d = gsl_odeiv2_driver_alloc_y_new(
        &sys, gsl_odeiv2_step_msadams, 3.3e-8, 1e-10, 0.0);

    double y[3] = { 1, 0, 0 }; // Начальные условия

    FILE* file;
    errno_t err;

    // Открытие файла с использованием fopen_s
    err = fopen_s(&file, "results.csv", "w");
    if (err != 0) {
        fprintf(stderr, "Не удалось открыть файл.\n");
        return -1;
    }

    fprintf(file, "t y1 y2 y3\n");

    // Запись результатов в файл
    for (double t = 0; t <= 1; t += 3.3e-4) {
        int status = gsl_odeiv2_driver_apply(d, &t, t + 3.3e-8, y);

        if (status != GSL_SUCCESS) {
            fprintf(stderr, "Ошибка при интегрировании: %s\n", gsl_strerror(status));
            break;
        }

        fprintf(file, "%.10f %.10f %.10f %.10f\n", t, y[0], y[1], y[2]);
    }

    // Закрытие файла
    fclose(file);
    gsl_odeiv2_driver_free(d);

    return 0;
}

//#include <gsl/gsl_matrix.h>
//#include <gsl/gsl_odeiv2.h>
//#include <stdio.h>
//#include <math.h>
//#include <gsl/gsl_odeiv2.h>
//#include <gsl/gsl_errno.h>
//#include <locale.h>
//
//// Функция системы ОДУ
//int func(double t, const double y[], double f[], void* params) {
//    (void)(t); // Используется для подавления предупреждений о неиспользуемых параметрах
//    // double lambda = *(double*)params;
//    double a = 60, b = -50, c = 0.1;
//    double lambda1 = a + 1.0 / (t + 1);
//    double lambda2 = b + 2.0 / (t + 1);
//    double lambda3 = c + 3.0 / (t + 1);
//
//    f[0] = lambda1 * y[0] + 1.0 / pow(t + 1, 4) * (b - a - 3.0 / (t + 1)) * y[1];
//    f[1] = lambda2 * y[1];
//    f[2] = 1.0 / pow(t + 1, 3) * (b - c - 4.0 / (t + 1)) * y[1] + lambda3 * y[2];
//
//    return GSL_SUCCESS;
//}
//
//// Функция для вычисления якобиана системы dy/dt = -100y
//int jacobian(double t, const double y[], double* dfdy, double dfdt[], void* params) {
//    gsl_matrix_view dfdy_mat = gsl_matrix_view_array(dfdy, 1, 1);
//    gsl_matrix* m = &dfdy_mat.matrix;
//    gsl_matrix_set(m, 0, 0, -100.0); // Значение производной dy/dt по y
//    dfdt[0] = 0.0; // Поскольку производная не зависит от времени t
//    return GSL_SUCCESS;
//}
//
//void solve_and_write_on_file(const char *file_name, const gsl_odeiv2_step_type* T);
//
//int main() 
//{
//    setlocale(LC_ALL, "");
//
//    solve_and_write_on_file("rk4_output.csv", gsl_odeiv2_step_rk4);
//
//    solve_and_write_on_file("adams_output.csv", gsl_odeiv2_step_msadams);
//    
//    return 0;
//}
//
//
//void solve_and_write_on_file(const char *file_name, const gsl_odeiv2_step_type * T)
//{
//    double y[3] = { 2.0, 1.0, 2.0}; // начальные условия
//
//    double t0 = 0.0, tk = 0.5;  // начальная и конечная точки интегрирования
//    double hstart = 5e-3; // величина шага
//    
//    
//    double minh = 1e-10, maxh = 0.0; // границы точности, левая и правая
//
//    gsl_odeiv2_system sys = { func, NULL, 3, NULL};
//    gsl_odeiv2_driver* d = gsl_odeiv2_driver_alloc_y_new(&sys, T, hstart, minh, maxh);
//    FILE* file;
//    errno_t err_rk4;
//    err_rk4 = fopen_s(&file, file_name, "w");
//
//    if (err_rk4 != 0) {
//        fprintf(stderr, "Не удалось открыть файлы.\n");
//        return;
//    }
//
//    fprintf(file, "t y1 y2 y3\n");
//
//    // Цикл интегрирования с фиксированным шагом
//    for (double ti = t0; ti <= tk; ti += hstart) {
//        int status_rk4 = gsl_odeiv2_driver_apply(d, &t0, ti, y);
//         
//        if (status_rk4 != GSL_SUCCESS) {
//            fprintf(stderr, "Ошибка при интегрировании: %s\n", gsl_strerror(status_rk4));
//            break;
//        }
//        fprintf(file, "%.10f %.10f %.10f %.10f\n", ti, y[0], y[1], y[2]);
//    }
//
//
//    // Закрытие файлов
//    fclose(file);
//    // Освобождение памяти
//    gsl_odeiv2_driver_free(d);
//}