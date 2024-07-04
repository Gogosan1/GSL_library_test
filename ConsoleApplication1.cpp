﻿#include <stdio.h>
#include <gsl/gsl_odeiv2.h>
#include <gsl/gsl_errno.h>

// Определение системы дифференциальных уравнений
int func(double t, const double y[], double f[], void* params) {
    f[0] = y[1];
    f[1] = -(y[1] / (t + 1) + (1 - 0.25 / ((t + 1) * (t + 1))) * y[0]);
    return GSL_SUCCESS;
}

int main() {
    // Начальные условия
    double y[2] = { 0.671396707, 0.0955400515 };
    double t = 0.0;
    double t1 = 20.0;
    double h = 1e-2; // Уменьшенный начальный шаг интегрирования

    // Инициализация метода Адамса
    const gsl_odeiv2_step_type* T = gsl_odeiv2_step_rk8pd;
    gsl_odeiv2_step* s = gsl_odeiv2_step_alloc(T, 2);
    gsl_odeiv2_control* c = gsl_odeiv2_control_y_new(1e-10, 0.0); // Установка точности
    gsl_odeiv2_evolve* e = gsl_odeiv2_evolve_alloc(2);
    gsl_odeiv2_system sys = { func, NULL, 2, NULL };

    // Открытие файла для записи результатов
    FILE* file;
    errno_t err;

    err = fopen_s(&file, "results_adams.csv", "w");
    if (err != 0) {
        printf("Ошибка при открытии файла.\n");
        return -1;
    }

    fprintf(file, "t y1 y2\n");
    // Процесс интегрирования
    while (t < t1) {
        int status = gsl_odeiv2_evolve_apply(e, c, s, &sys, &t, t1, &h, y);

        if (status != GSL_SUCCESS) {
            printf("Ошибка: статус = %d\n", status);
            break;
        }

        // Запись результатов в файл с точностью до 10 знаков после запятой
        fprintf(file, "%.10f %.10f %.10f\n", t, y[0], y[1]);
    }

    // Закрытие файла
    fclose(file);

    gsl_odeiv2_evolve_free(e);
    gsl_odeiv2_control_free(c);
    gsl_odeiv2_step_free(s);
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