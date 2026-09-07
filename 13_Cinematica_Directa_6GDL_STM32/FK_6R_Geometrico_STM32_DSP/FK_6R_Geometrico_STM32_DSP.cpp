// ============================================================================
// FK_6R_Geometrico_STM32_DSP.cpp
//
// Proyecto NUEVO (creado con el asistente de Keil, CMSIS-DSP como "Source",
// no como libreria precompilada -- para evitar el mismatch de FPU que causo
// el HardFault en el intento anterior). Mismo robot, mismos 6 casos de
// prueba, mismo reloj (216MHz) y misma medicion (TIM5) que la leccion 13,
// pero mide libm y CMSIS-DSP lado a lado en cada caso.
//
// Empieza con un DIAGNOSTICO paso a paso (igual idea que antes) antes de
// correr los 6 casos completos, para confirmar que CMSIS-DSP funciona bien
// con este nuevo setup antes de construir todo lo demas encima.
// ============================================================================
#include <stm32f7xx.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "arm_math.h"   // CMSIS-DSP -- arm_sin_f32 / arm_cos_f32 (Source, no Library)

#define PI 3.14159265358979323846

// ---------------------------------------------------------------------------
// Geometria del brazo (metros) -- tabla DH de la leccion 12
// ---------------------------------------------------------------------------
#define L1   0.065
#define L2   0.107
#define LD4  0.140   // L3+L4
#define LD6  0.173   // L5+L6
#define EPS_SINGULARIDAD 1e-6

typedef struct {
    double x, y, z;
    double yaw, pitch, roll;
} FK_Result;

typedef double Mat3[3][3];

void dh_rot(double theta, double alpha, Mat3 out) {
    double ct = cos(theta), st = sin(theta);
    double ca = cos(alpha), sa = sin(alpha);
    out[0][0] = ct;   out[0][1] = -st*ca;  out[0][2] =  st*sa;
    out[1][0] = st;   out[1][1] =  ct*ca;  out[1][2] = -ct*sa;
    out[2][0] = 0;    out[2][1] =  sa;     out[2][2] =  ca;
}

void mat3_mul(const Mat3 A, const Mat3 B, Mat3 out) {
    Mat3 tmp;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double s = 0;
            for (int k = 0; k < 3; k++) s += A[i][k]*B[k][j];
            tmp[i][j] = s;
        }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            out[i][j] = tmp[i][j];
}

void mat3_vec(const Mat3 R, const double v[3], double out[3]) {
    for (int i = 0; i < 3; i++) {
        double s = 0;
        for (int k = 0; k < 3; k++) s += R[i][k]*v[k];
        out[i] = s;
    }
}

FK_Result forward_kinematics(double theta1, double theta2, double theta3,
                              double theta4, double theta5, double theta6) {
    Mat3 R1, R2, R3, R4, R5, R6, R02, R03, R04, R05, R06;
    dh_rot(theta1,        PI/2,  R1);
    dh_rot(theta2,        0,     R2);
    dh_rot(theta3 + PI/2, PI/2,  R3);
    dh_rot(theta4 + PI/2, PI/2,  R4);
    dh_rot(theta5,       -PI/2,  R5);
    dh_rot(theta6,        0,     R6);
    mat3_mul(R1,R2,R02); mat3_mul(R02,R3,R03); mat3_mul(R03,R4,R04);
    mat3_mul(R04,R5,R05); mat3_mul(R05,R6,R06);

    Mat3 I3 = {{1,0,0},{0,1,0},{0,0,1}};
    double O[3] = {0,0,0};
    double p[3], Rp[3];

    p[0]=0; p[1]=0; p[2]=L1;
    mat3_vec(I3, p, Rp); O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=L2*cos(theta2); p[1]=L2*sin(theta2); p[2]=0;
    mat3_vec(R1, p, Rp); O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=0;
    mat3_vec(R02, p, Rp); O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=LD4;
    mat3_vec(R03, p, Rp); O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=0;
    mat3_vec(R04, p, Rp); O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=LD6;
    mat3_vec(R05, p, Rp); O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    double x = O[0], y = O[1], z = O[2];
    double R11=R06[0][0], R21=R06[1][0], R31=R06[2][0], R32=R06[2][1], R33=R06[2][2];
    double mag = sqrt(R11*R11 + R21*R21);
    double pitch = atan2(-R31, mag);
    double yaw, roll;
    if (mag < EPS_SINGULARIDAD) { yaw = 0.0; roll = PI; }
    else { yaw = atan2(R21, R11); roll = atan2(R32, R33); }

    FK_Result res = { x, y, z, yaw, pitch, roll };
    return res;
}

// ---------------------------------------------------------------------------
// dh_rot() con CMSIS-DSP (arm_cos_f32/arm_sin_f32) en vez de <math.h>.
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

FK_Result forward_kinematics_cmsis(double theta1, double theta2, double theta3,
                                    double theta4, double theta5, double theta6) {
    Mat3 R1, R2, R3, R4, R5, R6, R02, R03, R04, R05, R06;
    dh_rot_cmsis(theta1,        PI/2,  R1);
    dh_rot_cmsis(theta2,        0,     R2);
    dh_rot_cmsis(theta3 + PI/2, PI/2,  R3);
    dh_rot_cmsis(theta4 + PI/2, PI/2,  R4);
    dh_rot_cmsis(theta5,       -PI/2,  R5);
    dh_rot_cmsis(theta6,        0,     R6);
    mat3_mul(R1,R2,R02); mat3_mul(R02,R3,R03); mat3_mul(R03,R4,R04);
    mat3_mul(R04,R5,R05); mat3_mul(R05,R6,R06);

    Mat3 I3 = {{1,0,0},{0,1,0},{0,0,1}};
    double O[3] = {0,0,0};
    double p[3], Rp[3];

    p[0]=0; p[1]=0; p[2]=L1;
    mat3_vec(I3, p, Rp); O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=L2*cos(theta2); p[1]=L2*sin(theta2); p[2]=0;
    mat3_vec(R1, p, Rp); O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=0;
    mat3_vec(R02, p, Rp); O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=LD4;
    mat3_vec(R03, p, Rp); O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=0;
    mat3_vec(R04, p, Rp); O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    p[0]=0; p[1]=0; p[2]=LD6;
    mat3_vec(R05, p, Rp); O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

    double x = O[0], y = O[1], z = O[2];
    double R11=R06[0][0], R21=R06[1][0], R31=R06[2][0], R32=R06[2][1], R33=R06[2][2];
    double mag = sqrt(R11*R11 + R21*R21);
    double pitch = atan2(-R31, mag);
    double yaw, roll;
    if (mag < EPS_SINGULARIDAD) { yaw = 0.0; roll = PI; }
    else { yaw = atan2(R21, R11); roll = atan2(R32, R33); }

    FK_Result res = { x, y, z, yaw, pitch, roll };
    return res;
}

// ---------------------------------------------------------------------------
// Reloj a 216 MHz (identico a los otros proyectos del semillero)
// ---------------------------------------------------------------------------
void SystemClock_216MHz(void) {
    RCC->APB1ENR |= (1<<28);               // PWREN
    PWR->CR1 |= (0b11<<14);                // VOS = Scale 1
    PWR->CR1 |= (1<<16);                   // ODEN (Over-drive)
    while (!(PWR->CSR1 & (1<<16)));        // espera ODRDY
    PWR->CR1 |= (1<<17);                   // ODSWEN
    while (!(PWR->CSR1 & (1<<17)));        // espera ODSWRDY

    FLASH->ACR = 7 | (1<<8) | (1<<9);      // 7 wait states + prefetch + ART

    RCC->PLLCFGR = (16UL<<0) | (432UL<<6) | (0UL<<16) | (9UL<<24);
    RCC->CR |= (1<<24);
    while (!(RCC->CR & (1<<25)));

    RCC->CFGR |= (0b101<<10);              // APB1 = AHB/4 -> 54 MHz
    RCC->CFGR |= (0b100<<13);              // APB2 = AHB/2 -> 108 MHz
    RCC->CFGR |= (0b10<<0);                // SW = PLL
    while (((RCC->CFGR>>2) & 0b11) != 0b10);

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
    for (uint32_t i = 0; i < x; i++) SysTick_Wait(AHB_CLK_HZ / 1000);
}

void USART3_Init(void) {
    RCC->AHB1ENR |= (1<<3);                // GPIOD
    RCC->APB1ENR |= (1<<18);               // USART3
    GPIOD->MODER &= ~((0b11<<16) | (0b11<<18));
    GPIOD->MODER |= (0b10<<16) | (0b10<<18);
    GPIOD->AFR[1] &= ~((0b1111<<0) | (0b1111<<4));
    GPIOD->AFR[1] |= (7<<0) | (7<<4);
    USART3->BRR = APB1_CLK_HZ / 9600;
    USART3->CR1 = (1<<3) | (1<<0);
}
void USART3_SendChar(char c) {
    while (!((USART3->ISR>>7) & 1));
    USART3->TDR = c;
}
void USART3_SendString(const char *s) {
    while (*s) USART3_SendChar(*s++);
}

#define TIM5_CLK_MHZ 108.0

void TIM5_Init(void) {
    RCC->APB1ENR |= (1<<3);
    TIM5->PSC = 0;
    TIM5->ARR = 0xFFFFFFFF;
    TIM5->CNT = 0;
}

// ---------------------------------------------------------------------------
// DIAGNOSTICO -- corre PRIMERO, antes de los 6 casos completos. Prueba el
// calculo del Caso 2 (theta5=90, theta6=90 -- el primer +90 grados que se le
// pasa a arm_sin_f32/arm_cos_f32 en todo el programa) en 5 pasos separados.
// Si el chip se cuelga, el ultimo "DIAG N OK" impreso dice donde.
// ---------------------------------------------------------------------------
void Diagnostico_CMSIS_Caso2(void) {
    char buf[100];
    double t5 = 90.0 * PI / 180.0;
    double t6 = 90.0 * PI / 180.0;

    USART3_SendString("\r\n--- DIAGNOSTICO CMSIS-DSP (Caso 2: theta5=90, theta6=90) ---\r\n");

    float32_t s5 = arm_sin_f32((float32_t)t5);
    float32_t c5 = arm_cos_f32((float32_t)t5);
    snprintf(buf, sizeof(buf), "DIAG 1 OK -- arm_sin_f32(90)=%.6f arm_cos_f32(90)=%.6f\r\n", s5, c5);
    USART3_SendString(buf);

    float32_t s6 = arm_sin_f32((float32_t)t6);
    float32_t c6 = arm_cos_f32((float32_t)t6);
    snprintf(buf, sizeof(buf), "DIAG 2 OK -- arm_sin_f32(90)=%.6f arm_cos_f32(90)=%.6f (theta6)\r\n", s6, c6);
    USART3_SendString(buf);

    Mat3 R5;
    dh_rot_cmsis(t5, -PI/2, R5);
    snprintf(buf, sizeof(buf), "DIAG 3 OK -- dh_rot_cmsis(R5) hecha, R5[0][0]=%.4f\r\n", R5[0][0]);
    USART3_SendString(buf);

    Mat3 R6;
    dh_rot_cmsis(t6, 0, R6);
    snprintf(buf, sizeof(buf), "DIAG 4 OK -- dh_rot_cmsis(R6) hecha, R6[0][0]=%.4f\r\n", R6[0][0]);
    USART3_SendString(buf);

    FK_Result rc = forward_kinematics_cmsis(0, 0, 0, 0, t5, t6);
    snprintf(buf, sizeof(buf), "DIAG 5 OK -- forward_kinematics_cmsis(Caso2) completa: x=%.4f y=%.4f z=%.4f\r\n",
             rc.x, rc.y, rc.z);
    USART3_SendString(buf);

    USART3_SendString("--- DIAGNOSTICO TERMINADO SIN CUELGUE ---\r\n\r\n");
}

#define N_REPS 1000

void run_test_case(const char *nombre, double th1_deg, double th2_deg,
                    double th3_deg, double th4_deg, double th5_deg, double th6_deg) {
    double t1 = th1_deg * PI / 180.0, t2 = th2_deg * PI / 180.0, t3 = th3_deg * PI / 180.0;
    double t4 = th4_deg * PI / 180.0, t5 = th5_deg * PI / 180.0, t6 = th6_deg * PI / 180.0;
    char buf[220];

    TIM5->CNT = 0; TIM5->CR1 |= 1;
    FK_Result r = forward_kinematics(t1, t2, t3, t4, t5, t6);
    TIM5->CR1 &= ~1;
    uint32_t ticks_frio = TIM5->CNT;

    TIM5->CNT = 0; TIM5->CR1 |= 1;
    for (int i = 0; i < N_REPS; i++) r = forward_kinematics(t1, t2, t3, t4, t5, t6);
    TIM5->CR1 &= ~1;
    uint32_t ticks_prom = TIM5->CNT / N_REPS;

    TIM5->CNT = 0; TIM5->CR1 |= 1;
    FK_Result rc = forward_kinematics_cmsis(t1, t2, t3, t4, t5, t6);
    TIM5->CR1 &= ~1;
    uint32_t ticks_frio_c = TIM5->CNT;

    TIM5->CNT = 0; TIM5->CR1 |= 1;
    for (int i = 0; i < N_REPS; i++) rc = forward_kinematics_cmsis(t1, t2, t3, t4, t5, t6);
    TIM5->CR1 &= ~1;
    uint32_t ticks_prom_c = TIM5->CNT / N_REPS;

    snprintf(buf, sizeof(buf),
        "\r\n=== %s ===\r\n  entradas (grados): th1=%.1f th2=%.1f th3=%.1f th4=%.1f th5=%.1f th6=%.1f\r\n",
        nombre, th1_deg, th2_deg, th3_deg, th4_deg, th5_deg, th6_deg);
    USART3_SendString(buf);

    snprintf(buf, sizeof(buf), "  [libm]      x=%.4f y=%.4f z=%.4f  yaw=%.2f pitch=%.2f roll=%.2f\r\n",
        r.x, r.y, r.z, r.yaw*180.0/PI, r.pitch*180.0/PI, r.roll*180.0/PI);
    USART3_SendString(buf);

    snprintf(buf, sizeof(buf), "  [CMSIS-DSP] x=%.4f y=%.4f z=%.4f  yaw=%.2f pitch=%.2f roll=%.2f\r\n",
        rc.x, rc.y, rc.z, rc.yaw*180.0/PI, rc.pitch*180.0/PI, rc.roll*180.0/PI);
    USART3_SendString(buf);

    snprintf(buf, sizeof(buf), "  [libm]      ticks TIM5 (en frio) = %lu (%.3f us)   (promedio %d) = %lu (%.3f us)\r\n",
        (unsigned long)ticks_frio, ticks_frio / TIM5_CLK_MHZ, N_REPS, (unsigned long)ticks_prom, ticks_prom / TIM5_CLK_MHZ);
    USART3_SendString(buf);

    snprintf(buf, sizeof(buf), "  [CMSIS-DSP] ticks TIM5 (en frio) = %lu (%.3f us)   (promedio %d) = %lu (%.3f us)\r\n",
        (unsigned long)ticks_frio_c, ticks_frio_c / TIM5_CLK_MHZ, N_REPS, (unsigned long)ticks_prom_c, ticks_prom_c / TIM5_CLK_MHZ);
    USART3_SendString(buf);
}

void run_all_cases(void) {
    run_test_case("Caso 1 [extendido] q=0",                0,  0,   0,  0,  0,   0);
    run_test_case("Caso 2 [SINGULARIDAD del 6R] th5=90 th6=90", 0, 0, 0, 0, 90, 90);
    run_test_case("Caso 3 [generico]",                     30, 20, -15, 45, 60, -70);
    run_test_case("Caso EXTRA-1 [MEJOR CASO 'a ojo']",     90, 90, 90, 90, 90, 90);
    run_test_case("Caso EXTRA-2 [PEOR CASO]",              137.6, 23.9, 168.2, 74.5, 109.3, 41.7);
    run_test_case("Caso EXTRA-3 [MEJOR CASO REAL]",        0, 0, -90, -90, 90, 0);
}

void Boton_Init(void) {
    RCC->AHB1ENR |= (1<<2);
    GPIOC->MODER &= ~(0b11<<26);
    GPIOC->PUPDR &= ~(0b11<<26);
    GPIOC->PUPDR |= (0b01<<26);
}

int main(void) {
    SystemClock_216MHz();
    SCB_EnableICache();
    SCB_EnableDCache();
    TIM5_Init();
    USART3_Init();
    Boton_Init();

    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);

    USART3_SendString("\r\n\r\n=== FK_6R_Geometrico_STM32_DSP -- libm vs CMSIS-DSP (TIM5, 216MHz) ===\r\n");

    // Diagnostico primero -- confirma que CMSIS-DSP funciona bien con este
    // proyecto nuevo (Source, no Library) antes de correr los 6 casos.
    Diagnostico_CMSIS_Caso2();

    USART3_SendString("Presiona el boton de usuario (B1) para repetir los 6 casos.\r\n");
    run_all_cases();

    uint8_t boton_anterior = 1;
    while (1) {
        uint8_t boton_actual = (GPIOC->IDR >> 13) & 1;
        if (boton_anterior == 1 && boton_actual == 0) {
            SysTick_ms(30);
            if (((GPIOC->IDR >> 13) & 1) == 0) run_all_cases();
        }
        boton_anterior = boton_actual;
    }
}
