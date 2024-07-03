﻿#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>
#include <stdio.h>
#include <math.h>
#include <gsl/gsl_odeiv2.h>
#include <gsl/gsl_errno.h>

// Функция системы ОДУ
int func(double t, const double y[], double f[], void* params) {
    (void)(t); // Используется для подавления предупреждений о неиспользуемых параметрах
    double lambda = *(double*)params;
    f[0] = lambda * y[0];
    return GSL_SUCCESS;
}

// Функция для вычисления якобиана системы dy/dt = -100y
int jacobian(double t, const double y[], double* dfdy, double dfdt[], void* params) {
    gsl_matrix_view dfdy_mat = gsl_matrix_view_array(dfdy, 1, 1);
    gsl_matrix* m = &dfdy_mat.matrix;
    gsl_matrix_set(m, 0, 0, -100.0); // Значение производной dy/dt по y
    dfdt[0] = 0.0; // Поскольку производная не зависит от времени t
    return GSL_SUCCESS;
}

void solve_by_rk4_and_write_on_file(const char *file_name, double t0, double hstart, double lambda, const gsl_odeiv2_step_type* T, double y[], double tk, double minh, double maxh);

int main() 
{
    double lambda = -100.0; // Значение λ
    
   
    double y[1] = { 1.0 }; // начальные условия
   
    double t = 0.0, t1 = 1.0;  // начальная и конечная точки интегрирования
    double h = 1e-2; // величина шага
    double minh = 1e-2, maxh = 0.0; // допустимые значения шага
  

     solve_by_rk4_and_write_on_file("rk4_output.csv", t, h, lambda, gsl_odeiv2_step_rk4, y, t1, minh, maxh);

     y[0] = { 1.0 };
     t = 0.0, t1 = 1.0;  // начальная и конечная точки интегрирования
     h = 1e-2; // величина шага
     minh = 1e-2, maxh = 0.0; // допустимые значения шага


    solve_by_rk4_and_write_on_file("adams_output.csv", t, h,lambda, gsl_odeiv2_step_msadams, y, t1, minh, maxh);
    
    return 0;
}


void solve_by_rk4_and_write_on_file(const char *file_name, double t0, double hstart, double lambda, const gsl_odeiv2_step_type * T, double y[], double tk, double minh, double maxh)
{
    gsl_odeiv2_system sys = { func, NULL, 1, &lambda };
    gsl_odeiv2_driver* d = gsl_odeiv2_driver_alloc_y_new(&sys, T, hstart, minh, maxh);
    FILE* file;
    errno_t err_rk4;
    err_rk4 = fopen_s(&file, file_name, "w");

    if (err_rk4 != 0) {
        fprintf(stderr, "Не удалось открыть файлы.\n");
        return;
    }

    fprintf(file, "t y\n");

    // Цикл интегрирования с фиксированным шагом для rk4
    for (double ti = t0; ti <= tk; ti += 1e-2) {
        int status_rk4 = gsl_odeiv2_driver_apply(d, &t0, ti, y);
         
        if (status_rk4 != GSL_SUCCESS) {
            fprintf(stderr, "Ошибка при интегрировании: %s\n", gsl_strerror(status_rk4));
            break;
        }
        fprintf(file, "%.10f %.10f\n", ti, y[0]);
    }


    // Закрытие файлов
    fclose(file);
    // Освобождение памяти
    gsl_odeiv2_driver_free(d);
}