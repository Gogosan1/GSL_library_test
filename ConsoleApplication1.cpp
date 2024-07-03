#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>
#include <stdio.h>
#include <math.h>
#include <gsl/gsl_odeiv2.h>
#include <gsl/gsl_errno.h>

// Функция системы ОДУ
int func(double t, const double y[], double f[], void* params) {
    (void)(t); // Используется для подавления предупреждений о неиспользуемых параметрах
    // double lambda = *(double*)params;
    f[0] = 0.2*(y[1] - y[0]);
    f[1] = 10*y[0] - (60 - 0.125*y[2])*y[1] + 0.125*y[2];
    f[2] = 1;
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

void solve_and_write_on_file(const char *file_name,double lambda, const gsl_odeiv2_step_type* T);

int main() 
{
    double lambda = -100.0; // Значение λ
    
     solve_and_write_on_file("rk4_output.csv", lambda, gsl_odeiv2_step_rk4);

    solve_and_write_on_file("adams_output.csv", lambda, gsl_odeiv2_step_msadams);
    
    return 0;
}


void solve_and_write_on_file(const char *file_name, double lambda, const gsl_odeiv2_step_type * T)
{
    double y[3] = { 0.0, 0.0, 0.0}; // начальные условия

    double t0 = 0.0, tk = 200.0;  // начальная и конечная точки интегрирования
    double hstart = 1.7e-2; // величина шага
    double minh = 1e-10, maxh = 0.0; // границы точности, левая и правая

    gsl_odeiv2_system sys = { func, NULL, 3, NULL};
    gsl_odeiv2_driver* d = gsl_odeiv2_driver_alloc_y_new(&sys, T, hstart, minh, maxh);
    FILE* file;
    errno_t err_rk4;
    err_rk4 = fopen_s(&file, file_name, "w");

    if (err_rk4 != 0) {
        fprintf(stderr, "Не удалось открыть файлы.\n");
        return;
    }

    fprintf(file, "t y1 y2 y3\n");

    // Цикл интегрирования с фиксированным шагом
    for (double ti = t0; ti <= tk; ti += hstart) {
        int status_rk4 = gsl_odeiv2_driver_apply(d, &t0, ti, y);
         
        if (status_rk4 != GSL_SUCCESS) {
            fprintf(stderr, "Ошибка при интегрировании: %s\n", gsl_strerror(status_rk4));
            break;
        }
        fprintf(file, "%.10f %.10f %.10f %.10f\n", ti, y[0], y[1], y[2]);
    }


    // Закрытие файлов
    fclose(file);
    // Освобождение памяти
    gsl_odeiv2_driver_free(d);
}