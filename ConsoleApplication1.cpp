#include <stdio.h>
#include <math.h>
#include <gsl/gsl_odeiv2.h>
#include <gsl/gsl_errno.h>

// Функция системы ОДУ dy/dt = f(t, y)
int func(double t, const double y[], double f[], void* params) {
    double lambda = *(double*)params;
    f[0] = lambda * y[0];
    return GSL_SUCCESS;
}

// Запись результатов в файл
void write_results_to_file(FILE* file, double t, double y, double h) {
    fprintf(file, "%.10f %.10f %10f\n", t, y, h);
}

int main() {
    double lambda = -100.0; // Параметр lambda в ОДУ
    double y[1] = { 1.0 };  // Начальное условие y(0) = 1
    double t = 0.0;         // Начальное время
    double t1 = 1.0;        // Конечное время
    double h = 1e-2;        // Начальный шаг интегрирования h0

    // Инициализация метода, шага и драйвера
    gsl_odeiv2_step* s = gsl_odeiv2_step_alloc(gsl_odeiv2_step_rk4, 1);
    gsl_odeiv2_control* c = gsl_odeiv2_control_y_new(1e-8, 0.0); // Точность до 4 знаков после запятой
    gsl_odeiv2_evolve* e = gsl_odeiv2_evolve_alloc(1);
    gsl_odeiv2_system sys = { func, NULL, 1, &lambda };

    // Открытие файла для записи результатов
    FILE* file;
    errno_t err;

    err = fopen_s(&file, "results.csv", "w");
    if (err != 0) {
        fprintf(stderr, "Не удалось открыть файл.\n");
        return -1;
    }

    // Цикл интегрирования
    while (t < t1) {
        int status = gsl_odeiv2_evolve_apply(e, c, s, &sys, &t, t1, &h, y);

        if (status != GSL_SUCCESS) {
            fprintf(stderr, "Ошибка при интегрировании: %d\n", status);
            break;
        }

        write_results_to_file(file, t, y[0], h);
    }

    // Освобождение ресурсов и закрытие файла
    fclose(file);
    gsl_odeiv2_evolve_free(e);
    gsl_odeiv2_control_free(c);
    gsl_odeiv2_step_free(s);

    return 0;
}

// Фиксированное изменение шага
//#include <stdio.h>
//#include <gsl/gsl_odeiv2.h>
//#include <gsl/gsl_errno.h>
//
//// Определение функции системы ОДУ
//int func(double t, const double y[], double f[], void* params) {
//    double lambda = *(double*)params;
//    f[0] = lambda * y[0];
//    return GSL_SUCCESS;
//}
//
//// Главная функция
//int main() {
//    double lambda = -100.0; // Параметр системы
//    double y[1] = { 1.0 };  // Начальное условие
//    double t = 0.0;         // Начальное время
//    double t1 = 1.0;        // Конечное время
//    double h = 1e-2;        // Фиксированный шаг
//
//    gsl_odeiv2_step* s = gsl_odeiv2_step_alloc(gsl_odeiv2_step_rk4, 1);
//    gsl_odeiv2_system sys = { func, NULL, 1, &lambda };
//
//    FILE* file;
//    fopen_s(&file, "fixed_results.csv", "w");
//
//    for (double ti = t; ti < t1; ti += h) {
//        int status = gsl_odeiv2_step_apply(s, &ti, h, y, NULL, NULL, NULL, &sys);
//        if (status != GSL_SUCCESS) {
//            printf("Ошибка: %d\n", status);
//            break;
//        }
//        fprintf(file, "%.10f, %.10f\n", ti, y[0]);
//    }
//
//    fclose(file);
//    gsl_odeiv2_step_free(s);
//
//    return 0;
//}