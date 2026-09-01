// ============================================================================
// FK_6R_Geometrico_STM32_CMSIS.cpp
//
// Proyecto GEMELO de FK_6R_Geometrico_STM32.cpp (216MHz), pero DEDICADO al
// experimento libm vs CMSIS-DSP -- se separo a un archivo/proyecto Keil
// distinto a proposito, para no tocar el .cpp ya validado y documentado en
// la leccion 13 (README, secciones 8-11). Cinematica, geometria del brazo,
// casos de prueba, reloj (216MHz) y medicion (TIM5) son EXACTAMENTE iguales
// a FK_6R_Geometrico_STM32.cpp -- la UNICA diferencia real es que aqui el
// experimento de CMSIS-DSP esta ACTIVADO (EXPERIMENTO_CMSIS_DSP = 1).
//
// ================== PREGUNTA QUE RESPONDE ESTE PROYECTO ====================
// La leccion 13 (secciones 3 y 9) encontro que cos()/sin() de <math.h>
// (ARM Compiler 6 libm) NO tardan lo mismo para cualquier angulo -- tienen
// rutas rapidas para angulos "limpios" (0 grados, multiplos de 90 grados) y
// rutas lentas (reduccion de rango completa) para angulos genericos, lo que
// hace que el peor caso (Caso EXTRA-2, angulos irregulares) tarde ~49% mas
// que el mejor caso (Caso EXTRA-3, singularidad + angulos minimos). Los
// profesores preguntaron: ¿hay otra libreria para STM32 que SI de un tiempo
// ESTANDAR (constante), sin importar el angulo? La respuesta candidata es
// CMSIS-DSP: arm_sin_f32()/arm_cos_f32() usan una tabla precalculada +
// interpolacion lineal en vez de reduccion de rango + polinomio -- en teoria,
// tiempo fijo sin importar el angulo (a costa de menos precision, float en
// vez de double). Este proyecto mide si eso es cierto en la practica.
//
// ================== COMO ACTIVAR EL COMPONENTE CMSIS-DSP EN KEIL ===========
// Este .cpp YA tiene EXPERIMENTO_CMSIS_DSP = 1 y el codigo de
// forward_kinematics_cmsis() listo -- pero el proyecto Keil (clonado del
// original) TODAVIA NO tiene el componente CMSIS-DSP agregado. Paso manual
// obligatorio antes de compilar (una sola vez):
//   1. Abrir FK_6R_Geometrico_STM32_CMSIS.uvprojx en Keil uVision5.
//   2. Project > Manage Project Items... > Manage Run-Time Environment.
//   3. En el arbol, expandir "CMSIS" y marcar la casilla "DSP".
//   4. Keil pregunta que variante de libreria usar (o la resuelve solo segun
//      el FPU del dispositivo) -- aceptar la que proponga por defecto para
//      Cortex-M7 con FPU doble precision.
//   5. OK -- Keil agrega el include path y la libreria .lib correctas solas.
//      NO se edito el .uvprojx a mano para esto porque la version exacta de
//      la libreria depende del pack instalado -- mas seguro dejar que el
//      propio Keil la resuelva.
//   6. Build (F7). Si compila, Download y correr -- mismo protocolo de la
//      leccion 13 (HTerm/PuTTY, 9600 baudios, boton B1 repite los 6 casos).
// ============================================================================
#define EXPERIMENTO_CMSIS_DSP 1

#include <stm32f7xx.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#if EXPERIMENTO_CMSIS_DSP
#include "arm_math.h"   // CMSIS-DSP -- arm_sin_f32 / arm_cos_f32
#endif

#define PI 3.14159265358979323846

// ---------------------------------------------------------------------------
// Geometria del brazo (metros) -- tabla DH de la leccion 12 (identica)
// ---------------------------------------------------------------------------
#define L1   0.065
#define L2   0.107
#define LD4  0.140   // L3+L4 = 0.095+0.045 (el sistema 3 coincide con el 2, "regla 4")
#define LD6  0.173   // L5+L6 = 0.07+0.103  (el sistema 5 coincide con el 4, "regla 4")

// Umbral para detectar gimbal lock (|cos(pitch)| por debajo de esto se
// considera singularidad). Igual criterio que el brazo de 5R.
#define EPS_SINGULARIDAD 1e-6

typedef struct {
    double x, y, z;          // metros
    double yaw, pitch, roll; // radianes
} FK_Result;

typedef double Mat3[3][3];

// R_i = Rz(theta) * Rx(alpha) -- forma generica de una matriz de rotacion DH,
// igual formula que Metodo_Geometrico_RPY_6R.m -- version libm (cos/sin de
// <math.h>), identica a FK_6R_Geometrico_STM32.cpp.
void dh_rot(double theta, double alpha, Mat3 out) {
    double ct = cos(theta), st = sin(theta);
    double ca = cos(alpha), sa = sin(alpha);
    out[0][0] = ct;   out[0][1] = -st*ca;  out[0][2] =  st*sa;
    out[1][0] = st;   out[1][1] =  ct*ca;  out[1][2] = -ct*sa;
    out[2][0] = 0;    out[2][1] =  sa;     out[2][2] =  ca;
}

void mat3_mul(const Mat3 A, const Mat3 B, Mat3 out) {
    Mat3 tmp;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double s = 0;
            for (int k = 0; k < 3; k++) s += A[i][k]*B[k][j];
            tmp[i][j] = s;
        }
    }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            out[i][j] = tmp[i][j];
}

// out = R * v (matriz 3x3 por vector 3x1)
void mat3_vec(const Mat3 R, const double v[3], double out[3]) {
    for (int i = 0; i < 3; i++) {
        double s = 0;
        for (int k = 0; k < 3; k++) s += R[i][k]*v[k];
        out[i] = s;
    }
}

// ---------------------------------------------------------------------------
// Cinematica directa geometrica, brazo de 6 GDL real -- version libm.
// Identica a FK_6R_Geometrico_STM32.cpp (ver ese archivo / README leccion 13
// para la explicacion completa de cada parte). Sirve como referencia (el
// "Caso base") contra la que se compara la version CMSIS-DSP mas abajo.
// ---------------------------------------------------------------------------
FK_Result forward_kinematics(double theta1, double theta2, double theta3,
                              double theta4, double theta5, double theta6) {
    Mat3 R1, R2, R3, R4, R5, R6, R02, R03, R04, R05, R06;
    dh_rot(theta1,          PI/2,  R1);
    dh_rot(theta2,          0,     R2);
    dh_rot(theta3 + PI/2,   PI/2,  R3);
    dh_rot(theta4 + PI/2,   PI/2,  R4);
    dh_rot(theta5,         -PI/2,  R5);
    dh_rot(theta6,          0,     R6);

    mat3_mul(R1, R2, R02);
    mat3_mul(R02, R3, R03);
    mat3_mul(R03, R4, R04);
    mat3_mul(R04, R5, R05);
    mat3_mul(R05, R6, R06);

    Mat3 I3 = {{1,0,0},{0,1,0},{0,0,1}};
    double O[3] = {0,0,0};
    double p[3], Rp[3];

    p[0]=0; p[1]=0; p[2]=L1;
    mat3_vec(I3, p, Rp);
    O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=L2*cos(theta2); p[1]=L2*sin(theta2); p[2]=0;
    mat3_vec(R1, p, Rp);
    O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=0;
    mat3_vec(R02, p, Rp);
    O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=LD4;
    mat3_vec(R03, p, Rp);
    O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=0;
    mat3_vec(R04, p, Rp);
    O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=LD6;
    mat3_vec(R05, p, Rp);
    O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    double x = O[0], y = O[1], z = O[2];

    double R11 = R06[0][0], R21 = R06[1][0], R31 = R06[2][0];
    double R32 = R06[2][1], R33 = R06[2][2];

    double mag   = sqrt(R11*R11 + R21*R21); // = |cos(pitch)|
    double pitch = atan2(-R31, mag);
    double yaw, roll;

    if (mag < EPS_SINGULARIDAD) {
        yaw  = 0.0;
        roll = PI; // 180 grados
    } else {
        yaw  = atan2(R21, R11);
        roll = atan2(R32, R33);
    }

    FK_Result res = { x, y, z, yaw, pitch, roll };
    return res;
}

// ---------------------------------------------------------------------------
// dh_rot(), pero con arm_cos_f32()/arm_sin_f32() de CMSIS-DSP en vez de
// cos()/sin() de <math.h> -- ESTE es el experimento real de este archivo.
// arm_cos_f32/arm_sin_f32 trabajan en float (float32_t), no double -- se
// convierte al entrar y al guardar en la matriz (Mat3 sigue siendo double,
// para poder reusar mat3_mul/mat3_vec tal cual, sin duplicarlos).
// ---------------------------------------------------------------------------
void dh_rot_cmsis(double theta, double alpha, Mat3 out) {
    float32_t ct = arm_cos_f32((float32_t)theta);
    float32_t st = arm_sin_f32((float32_t)theta);
    float32_t ca = arm_cos_f32((float32_t)alpha);
    float32_t sa = arm_sin_f32((float32_t)alpha);
    out[0][0] = ct;   out[0][1] = -st*ca;  out[0][2] =  st*sa;
    out[1][0] = st;   out[1][1] =  ct*ca;  out[1][2] = -ct*sa;
    out[2][0] = 0;    out[2][1] =  sa;     out[2][2] =  ca;
}

// forward_kinematics(), identica en todo excepto que arma R1..R6 con
// dh_rot_cmsis() en vez de dh_rot() -- mat3_mul/mat3_vec (pura aritmetica,
// sin trigonometria) quedan exactamente iguales, y yaw/pitch/roll se siguen
// extrayendo con atan2()/sqrt() de <math.h> (eso NO es lo que se esta
// comparando aqui -- el experimento es solo sobre cos()/sin()).
FK_Result forward_kinematics_cmsis(double theta1, double theta2, double theta3,
                                    double theta4, double theta5, double theta6) {
    Mat3 R1, R2, R3, R4, R5, R6, R02, R03, R04, R05, R06;
    dh_rot_cmsis(theta1,          PI/2,  R1);
    dh_rot_cmsis(theta2,          0,     R2);
    dh_rot_cmsis(theta3 + PI/2,   PI/2,  R3);
    dh_rot_cmsis(theta4 + PI/2,   PI/2,  R4);
    dh_rot_cmsis(theta5,         -PI/2,  R5);
    dh_rot_cmsis(theta6,          0,     R6);

    mat3_mul(R1, R2, R02);
    mat3_mul(R02, R3, R03);
    mat3_mul(R03, R4, R04);
    mat3_mul(R04, R5, R05);
    mat3_mul(R05, R6, R06);

    Mat3 I3 = {{1,0,0},{0,1,0},{0,0,1}};
    double O[3] = {0,0,0};
    double p[3], Rp[3];

    p[0]=0; p[1]=0; p[2]=L1;
    mat3_vec(I3, p, Rp);
    O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=L2*cos(theta2); p[1]=L2*sin(theta2); p[2]=0;
    mat3_vec(R1, p, Rp);
    O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=0;
    mat3_vec(R02, p, Rp);
    O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=LD4;
    mat3_vec(R03, p, Rp);
    O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=0;
    mat3_vec(R04, p, Rp);
    O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=LD6;
    mat3_vec(R05, p, Rp);
    O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    double x = O[0], y = O[1], z = O[2];

    double R11 = R06[0][0], R21 = R06[1][0], R31 = R06[2][0];
    double R32 = R06[2][1], R33 = R06[2][2];

    double mag   = sqrt(R11*R11 + R21*R21);
    double pitch = atan2(-R31, mag);
    double yaw, roll;

    if (mag < EPS_SINGULARIDAD) {
        yaw  = 0.0;
        roll = PI;
    } else {
        yaw  = atan2(R21, R11);
        roll = atan2(R32, R33);
    }

    FK_Result res = { x, y, z, yaw, pitch, roll };
    return res;
}

// ---------------------------------------------------------------------------
// Reloj a 216 MHz -- identico a FK_6R_Geometrico_STM32.cpp (mismo Over-drive,
// misma cadena de PLL). Se corre a la velocidad maxima porque es donde mas
// se nota cualquier diferencia entre libm y CMSIS-DSP en microsegundos.
// ---------------------------------------------------------------------------
void SystemClock_216MHz(void) {
    RCC->APB1ENR |= (1<<28);               // PWREN
    PWR->CR1 |= (0b11<<14);                // VOS = Scale 1
    PWR->CR1 |= (1<<16);                   // ODEN (Over-drive)
    while (!(PWR->CSR1 & (1<<16)));        // espera ODRDY
    PWR->CR1 |= (1<<17);                   // ODSWEN (conmuta a Over-drive)
    while (!(PWR->CSR1 & (1<<17)));        // espera ODSWRDY

    FLASH->ACR = 7 | (1<<8) | (1<<9);      // 7 wait states + prefetch + ART

    // HSI=16MHz -> PLLM=16 -> 1MHz -> PLLN=432 -> 432MHz -> PLLP=2 -> 216MHz
    RCC->PLLCFGR = (16UL<<0) | (432UL<<6) | (0UL<<16) | (9UL<<24); // PLLSRC=0 -> HSI

    RCC->CR |= (1<<24);                    // PLLON
    while (!(RCC->CR & (1<<25)));          // espera PLLRDY

    RCC->CFGR |= (0b101<<10);              // APB1 = AHB/4  -> 54 MHz
    RCC->CFGR |= (0b100<<13);              // APB2 = AHB/2  -> 108 MHz

    RCC->CFGR |= (0b10<<0);                // SW = PLL
    while (((RCC->CFGR>>2) & 0b11) != 0b10); // espera SWS = PLL

    SystemCoreClock = 216000000UL;
}

#define AHB_CLK_HZ   216000000UL
#define APB1_CLK_HZ  54000000UL

void SysTick_Wait(uint32_t n) {
    SysTick->LOAD = n - 1;
    SysTick->VAL = 0;
    while (((SysTick->CTRL & 0x00010000) >> 16) == 0);
}
void SysTick_ms(uint32_t x) {
    for (uint32_t i = 0; i < x; i++) {
        SysTick_Wait(AHB_CLK_HZ / 1000);
    }
}

// ---------------------------------------------------------------------------
// USART3 (PD8=TX, PD9=RX, AF7) -- VCP del ST-LINK en las Nucleo-144.
// ---------------------------------------------------------------------------
void USART3_Init(void) {
    RCC->AHB1ENR |= (1<<3);                // GPIOD
    RCC->APB1ENR |= (1<<18);               // USART3

    GPIOD->MODER &= ~((0b11<<16) | (0b11<<18));
    GPIOD->MODER |= (0b10<<16) | (0b10<<18);         // AF mode en PD8, PD9
    GPIOD->AFR[1] &= ~((0b1111<<0) | (0b1111<<4));
    GPIOD->AFR[1] |= (7<<0) | (7<<4);                // AF7 = USART3

    USART3->BRR = APB1_CLK_HZ / 9600;
    USART3->CR1 = (1<<3) | (1<<0);         // TE, UE
}

void USART3_SendChar(char c) {
    while (!((USART3->ISR>>7) & 1));       // espera TXE
    USART3->TDR = c;
}
void USART3_SendString(const char *s) {
    while (*s) USART3_SendChar(*s++);
}

// ---------------------------------------------------------------------------
// TIM5 -- misma medicion de tiempo que la leccion 13 (TIM5 a 108MHz a
// 216MHz, ver seccion 6 del README para la explicacion del doblado de reloj).
// ---------------------------------------------------------------------------
#define TIM5_CLK_MHZ 108.0

void TIM5_Init(void) {
    RCC->APB1ENR |= (1<<3);                // TIM5EN
    TIM5->PSC = 0;                         // sin division, resolucion maxima
    TIM5->ARR = 0xFFFFFFFF;                // maximo (32 bits)
    TIM5->CNT = 0;
}

#define N_REPS 1000  // repeticiones para el promedio en estado estable (cache caliente)

void run_test_case(const char *nombre, double th1_deg, double th2_deg,
                    double th3_deg, double th4_deg, double th5_deg, double th6_deg) {
    double t1 = th1_deg * PI / 180.0;
    double t2 = th2_deg * PI / 180.0;
    double t3 = th3_deg * PI / 180.0;
    double t4 = th4_deg * PI / 180.0;
    double t5 = th5_deg * PI / 180.0;
    double t6 = th6_deg * PI / 180.0;

    char buf[220];

    // ---- version 1: libm de <math.h> (cos()/sin()/atan2(), reduccion de rango) ----
    TIM5->CNT = 0;
    TIM5->CR1 |= (1<<0);                   // arranca el conteo
    FK_Result r = forward_kinematics(t1, t2, t3, t4, t5, t6);
    TIM5->CR1 &= ~(1<<0);                  // para el conteo
    uint32_t ticks_frio = TIM5->CNT;

    TIM5->CNT = 0;
    TIM5->CR1 |= (1<<0);
    for (int i = 0; i < N_REPS; i++) {
        r = forward_kinematics(t1, t2, t3, t4, t5, t6);
    }
    TIM5->CR1 &= ~(1<<0);
    uint32_t ticks_prom = TIM5->CNT / N_REPS;

    // ---- version 2: CMSIS-DSP (arm_cos_f32()/arm_sin_f32(), tabla + interpolacion) ----
    TIM5->CNT = 0;
    TIM5->CR1 |= (1<<0);
    FK_Result rc = forward_kinematics_cmsis(t1, t2, t3, t4, t5, t6);
    TIM5->CR1 &= ~(1<<0);
    uint32_t ticks_frio_c = TIM5->CNT;

    TIM5->CNT = 0;
    TIM5->CR1 |= (1<<0);
    for (int i = 0; i < N_REPS; i++) {
        rc = forward_kinematics_cmsis(t1, t2, t3, t4, t5, t6);
    }
    TIM5->CR1 &= ~(1<<0);
    uint32_t ticks_prom_c = TIM5->CNT / N_REPS;

    snprintf(buf, sizeof(buf),
        "\r\n=== %s ===\r\n"
        "  entradas (grados): th1=%.1f th2=%.1f th3=%.1f th4=%.1f th5=%.1f th6=%.1f\r\n",
        nombre, th1_deg, th2_deg, th3_deg, th4_deg, th5_deg, th6_deg);
    USART3_SendString(buf);

    snprintf(buf, sizeof(buf),
        "  [libm]      x=%.4f y=%.4f z=%.4f  yaw=%.2f pitch=%.2f roll=%.2f\r\n",
        r.x, r.y, r.z, r.yaw*180.0/PI, r.pitch*180.0/PI, r.roll*180.0/PI);
    USART3_SendString(buf);

    snprintf(buf, sizeof(buf),
        "  [CMSIS-DSP] x=%.4f y=%.4f z=%.4f  yaw=%.2f pitch=%.2f roll=%.2f\r\n",
        rc.x, rc.y, rc.z, rc.yaw*180.0/PI, rc.pitch*180.0/PI, rc.roll*180.0/PI);
    USART3_SendString(buf);

    snprintf(buf, sizeof(buf),
        "  [libm]      ticks TIM5 (en frio) = %lu (%.3f us)   (promedio %d) = %lu (%.3f us)\r\n",
        (unsigned long)ticks_frio, ticks_frio / TIM5_CLK_MHZ,
        N_REPS, (unsigned long)ticks_prom, ticks_prom / TIM5_CLK_MHZ);
    USART3_SendString(buf);

    snprintf(buf, sizeof(buf),
        "  [CMSIS-DSP] ticks TIM5 (en frio) = %lu (%.3f us)   (promedio %d) = %lu (%.3f us)\r\n",
        (unsigned long)ticks_frio_c, ticks_frio_c / TIM5_CLK_MHZ,
        N_REPS, (unsigned long)ticks_prom_c, ticks_prom_c / TIM5_CLK_MHZ);
    USART3_SendString(buf);
}

void run_all_cases(void) {
    // Mismos 6 casos de la leccion 13 (secciones 8-10) -- incluidos los 3
    // EXTRA (mejor caso "a ojo", peor caso realista, y el mejor caso real
    // EXTRA-3 encontrado por sympy) para poder comparar libm vs CMSIS-DSP
    // exactamente en los mismos puntos donde ya se caracterizo el rango de
    // tiempo con libm solo.
    run_test_case("Caso 1 [extendido] q=0",                0,  0,   0,  0,  0,   0);
    run_test_case("Caso 2 [SINGULARIDAD del 6R] th5=90 th6=90", 0, 0, 0, 0, 90, 90);
    run_test_case("Caso 3 [generico]",                     30, 20, -15, 45, 60, -70);
    run_test_case("Caso EXTRA-1 [MEJOR CASO 'a ojo': los 6 angulos en 90 grados]",
                  90, 90, 90, 90, 90, 90);
    run_test_case("Caso EXTRA-2 [PEOR CASO: angulos irregulares, dentro de 0-180]",
                  137.6, 23.9, 168.2, 74.5, 109.3, 41.7);
    run_test_case("Caso EXTRA-3 [MEJOR CASO REAL: singularidad + min. angulos]",
                  0, 0, -90, -90, 90, 0);
}

// ---------------------------------------------------------------------------
// Boton de usuario (PC13) -- repite los 6 casos cada vez que se presiona.
// ---------------------------------------------------------------------------
void Boton_Init(void) {
    RCC->AHB1ENR |= (1<<2);                // GPIOC

    GPIOC->MODER &= ~(0b11<<26);           // PC13 entrada
    GPIOC->PUPDR &= ~(0b11<<26);
    GPIOC->PUPDR |= (0b01<<26);            // pull-up (B1 en Nucleo-144 no
                                            // trae resistencia externa; idle=1,
                                            // presionado=0)
}

int main(void) {
    SystemClock_216MHz();

    SCB_EnableICache();                    // I-cache y D-cache del Cortex-M7
    SCB_EnableDCache();

    TIM5_Init();
    USART3_Init();
    Boton_Init();

    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);

    USART3_SendString("\r\n\r\n=== Cinematica Directa 6 GDL -- STM32F767ZI @ 216MHz -- libm vs CMSIS-DSP (TIM5) ===\r\n");
    USART3_SendString("Presiona el boton de usuario (B1) para repetir los 6 casos.\r\n");
    run_all_cases();

    uint8_t boton_anterior = 1; // idle = 1 (pull-up)
    while (1) {
        uint8_t boton_actual = (GPIOC->IDR >> 13) & 1;
        if (boton_anterior == 1 && boton_actual == 0) { // flanco de bajada = presionado
            SysTick_ms(30); // antirrebote
            if (((GPIOC->IDR >> 13) & 1) == 0) {
                run_all_cases();
            }
        }
        boton_anterior = boton_actual;
    }
}
