// ============================================================================
// FK_6R_Geometrico_STM32_HSI16MHz.cpp
//
// MISMO codigo de cinematica directa (metodo geometrico, 6 GDL real) que
// FK_6R_Geometrico_STM32.cpp -- la UNICA diferencia entre los dos proyectos
// es el reloj: este corre a la velocidad "de fabrica" (HSI interno, 16MHz,
// SIN PLL), el otro corre a 216MHz (maxima velocidad del STM32F767ZI, PLL
// con Over-drive). Comparar los ciclos/tiempo de los dos da el efecto puro
// de subir el reloj, sin cambiar ni una linea de la logica de calculo.
//
// ================== REVISION 2 (correccion del profesor) ==================
// Mismos dos cambios que FK_6R_Geometrico_STM32.cpp -- ver ese archivo para
// la explicacion completa:
// 1) Posicion: metodo recursivo eslabon-por-eslabon (Inversa2R.pdf del
//    profesor) en vez del atajo trigonometrico anterior.
// 2) Medicion de tiempo: TIM5 en vez de DWT->CYCCNT.
// ============================================================================
//
// Reloj:  HSI directo, 16 MHz, sin PLL. AHB=APB1=APB2=16MHz (sin prescalers,
//         es el estado por defecto al resetear el chip -- se deja explicito
//         aqui solo para que quede documentado, no porque haga falta).
// Medicion: TIM5->CNT, con TIM5 corriendo a 16MHz (APB1=16MHz CON prescaler
//          de bus =/1 -> el reloj de TIM NO se dobla, a diferencia de la
//          version de 216MHz -- regla estandar del arbol de reloj del F7).
// Salida:  USART3 (PD8=TX, PD9=RX, AF7) -> puerto virtual COM del ST-LINK.
//         9600 baudios, BRR recalculado para APB1=16MHz (antes era 54MHz).
// ============================================================================

#include <stm32f7xx.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define PI 3.14159265358979323846

// ---------------------------------------------------------------------------
// Geometria del brazo (metros) -- tabla DH de la leccion 12 (identica)
// ---------------------------------------------------------------------------
#define L1   0.065
#define L2   0.107
#define LD4  0.140   // L3+L4 = 0.095+0.045 (el sistema 3 coincide con el 2, "regla 4")
#define LD6  0.173   // L5+L6 = 0.07+0.103  (el sistema 5 coincide con el 4, "regla 4")

#define EPS_SINGULARIDAD 1e-6

typedef struct {
    double x, y, z;          // metros
    double yaw, pitch, roll; // radianes
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
// Cinematica directa geometrica, brazo de 6 GDL real (IDENTICA a la version
// de 216MHz -- ver ese archivo para la explicacion completa de cada parte).
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

    // Posicion: O_i = O_(i-1) + R_(i-1)^0 * p_i , eslabon por eslabon.
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
// Reloj a 16 MHz -- HSI directo, SIN PLL. En un STM32F7 recien reseteado
// SW ya esta en HSI por defecto (esto queda aqui solo para documentar la
// intencion explicitamente, no porque haga falta tocar ningun registro).
// ---------------------------------------------------------------------------
void SystemClock_HSI16MHz(void) {
    RCC->CR |= (1<<0);                     // HSION (por si acaso)
    while (!(RCC->CR & (1<<1)));           // espera HSIRDY

    RCC->CFGR &= ~(0b11<<0);               // SW = HSI (000)
    while (((RCC->CFGR>>2) & 0b11) != 0b00); // espera SWS = HSI

    // HPRE, PPRE1, PPRE2 se dejan en /1 (reset por defecto) -> AHB=APB1=APB2=16MHz
    SystemCoreClock = 16000000UL;
}

#define AHB_CLK_HZ   16000000UL
#define APB1_CLK_HZ  16000000UL

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
// BRR recalculado para APB1=16MHz (antes 54MHz a 216MHz de reloj de sistema).
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
// TIM5 -- medicion de tiempo (reemplaza a DWT->CYCCNT, pedido del profesor).
// PSC=0 (resolucion maxima). Aqui APB1=16MHz CON prescaler de bus =/1
// (HPRE/PPRE1 en reset por defecto), asi que el reloj de TIM5 NO se dobla:
// TIM5 corre a 16MHz -- distinto a la version de 216MHz, donde TIM5 SI se
// dobla a 108MHz porque ahi el prescaler de APB1 es /4.
// ---------------------------------------------------------------------------
#define TIM5_CLK_MHZ 16.0

void TIM5_Init(void) {
    RCC->APB1ENR |= (1<<3);                // TIM5EN
    TIM5->PSC = 0;                         // sin division, resolucion maxima
    TIM5->ARR = 0xFFFFFFFF;                // maximo (32 bits)
    TIM5->CNT = 0;
}

#define N_REPS 1000

void run_test_case(const char *nombre, double th1_deg, double th2_deg,
                    double th3_deg, double th4_deg, double th5_deg, double th6_deg) {
    double t1 = th1_deg * PI / 180.0;
    double t2 = th2_deg * PI / 180.0;
    double t3 = th3_deg * PI / 180.0;
    double t4 = th4_deg * PI / 180.0;
    double t5 = th5_deg * PI / 180.0;
    double t6 = th6_deg * PI / 180.0;

    char buf[180];

    TIM5->CNT = 0;
    TIM5->CR1 |= (1<<0);
    FK_Result r = forward_kinematics(t1, t2, t3, t4, t5, t6);
    TIM5->CR1 &= ~(1<<0);
    uint32_t ticks_frio = TIM5->CNT;

    TIM5->CNT = 0;
    TIM5->CR1 |= (1<<0);
    for (int i = 0; i < N_REPS; i++) {
        r = forward_kinematics(t1, t2, t3, t4, t5, t6);
    }
    TIM5->CR1 &= ~(1<<0);
    uint32_t ticks_prom = TIM5->CNT / N_REPS;

    snprintf(buf, sizeof(buf),
        "\r\n=== %s ===\r\n"
        "  entradas (grados): th1=%.1f th2=%.1f th3=%.1f th4=%.1f th5=%.1f th6=%.1f\r\n",
        nombre, th1_deg, th2_deg, th3_deg, th4_deg, th5_deg, th6_deg);
    USART3_SendString(buf);

    snprintf(buf, sizeof(buf),
        "  x=%.4f m  y=%.4f m  z=%.4f m\r\n"
        "  yaw=%.2f deg  pitch=%.2f deg  roll=%.2f deg\r\n",
        r.x, r.y, r.z, r.yaw*180.0/PI, r.pitch*180.0/PI, r.roll*180.0/PI);
    USART3_SendString(buf);

    snprintf(buf, sizeof(buf),
        "  ticks TIM5 (1 ejecucion, en frio)    = %lu  (%.3f us)\r\n"
        "  ticks TIM5 (promedio %d ejecuciones) = %lu  (%.3f us)\r\n",
        (unsigned long)ticks_frio, ticks_frio / TIM5_CLK_MHZ,
        N_REPS, (unsigned long)ticks_prom, ticks_prom / TIM5_CLK_MHZ);
    USART3_SendString(buf);
}

void run_all_cases(void) {
    run_test_case("Caso 1 [extendido] q=0",                0,  0,   0,  0,  0,   0);
    run_test_case("Caso 2 [SINGULARIDAD del 6R] th5=90 th6=90", 0, 0, 0, 0, 90, 90);
    run_test_case("Caso 3 [generico]",                     30, 20, -15, 45, 60, -70);
}

void Boton_Init(void) {
    RCC->AHB1ENR |= (1<<2);                // GPIOC

    GPIOC->MODER &= ~(0b11<<26);           // PC13 entrada
    GPIOC->PUPDR &= ~(0b11<<26);
    GPIOC->PUPDR |= (0b01<<26);            // pull-up (B1 no trae resistencia externa)
}

int main(void) {
    SystemClock_HSI16MHz();

    SCB_EnableICache();
    SCB_EnableDCache();

    TIM5_Init();
    USART3_Init();
    Boton_Init();

    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);

    USART3_SendString("\r\n\r\n=== Cinematica Directa 6 GDL real -- STM32F767ZI @ 16MHz (HSI, sin PLL, TIM5) ===\r\n");
    USART3_SendString("Presiona el boton de usuario (B1) para repetir las 3 pruebas.\r\n");
    run_all_cases();

    uint8_t boton_anterior = 1;
    while (1) {
        uint8_t boton_actual = (GPIOC->IDR >> 13) & 1;
        if (boton_anterior == 1 && boton_actual == 0) {
            SysTick_ms(30);
            if (((GPIOC->IDR >> 13) & 1) == 0) {
                run_all_cases();
            }
        }
        boton_anterior = boton_actual;
    }
}
