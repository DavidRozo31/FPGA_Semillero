// ============================================================================
// FK_6R_Geometrico_STM32.cpp
//
// Cinematica directa (metodo geometrico) del brazo de 6 GDL REAL (ya no el de
// 5 articulaciones + roll de gripper del proyecto FK_5R_Geometrico_STM32).
// Tabla DH corregida por el profesor ("regla 4": sistemas 3 y 5 coinciden con
// 2 y 4) -- ver leccion 12 del repo del semillero para la derivacion completa.
//
// Diferencia clave respecto al brazo de 5R: aqui la POSICION deja de ser
// independiente de la muñeca (theta4/theta5 SI mueven x,y,z porque la muñeca
// tiene longitud fisica propia, Ld4 y Ld6) -- por eso la posicion se calcula
// en dos partes (x4,y4,z4 hasta el sistema 4, mas el aporte de Ld6 en la
// direccion de Z6 = columna 3 de R0_6).
//
// Objetivo: medir cuantos ciclos de CPU tarda el STM32 en la cinematica
// directa completa del 6R, para comparar contra el diseño de FPGA (que paso
// de una version en paralelo con multiplicadores que no cabia en el chip, a
// una version que reusa un solo multiplicador en serie: ~152-160 ciclos a
// 50MHz -> ~3.0-3.2us de un start_in a un done_out).
//
// Reloj:  PLL a 216 MHz (igual que FK_5R_Geometrico_STM32).
// Medicion: contador de ciclos DWT->CYCCNT (1 ciclo de resolucion).
// Salida:  USART3 (PD8=TX, PD9=RX, AF7) -> puerto virtual COM del ST-LINK.
//         9600 baudios, BRR calculado para APB1=54MHz.
// ============================================================================

#include <stm32f7xx.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define PI 3.14159265358979323846

// ---------------------------------------------------------------------------
// Geometria del brazo (metros) -- tabla DH de la leccion 12
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
// igual formula que Metodo_Geometrico_RPY_6R.m
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

// ---------------------------------------------------------------------------
// Cinematica directa geometrica, brazo de 6 GDL real.
//
// Posicion: PARTE A (formula cerrada, igual espiritu que el brazo de 5R pero
// solo hasta el sistema 4, sin termino phi234 porque theta4 ya no es
// coplanar con theta2,theta3 en esta tabla) + PARTE B (aporte de la muñeca,
// Ld6 en la direccion de Z6 en el mundo = columna 3 de R0_6 -- ver leccion 12
// seccion 3 para la derivacion completa de por que).
//
// Orientacion: cadena de matrices R1*R2*R3*R4*R5*R6 (igual que
// Metodo_Geometrico_RPY_6R.m), NO la formula cerrada reducida con sympy que
// usa la FPGA -- por la misma razon que en el brazo de 5R: la cadena de
// matrices conserva el residuo de punto flotante de cos(pi/2) que resuelve
// el reparto de la singularidad de forma consistente con MATLAB.
// ---------------------------------------------------------------------------
FK_Result forward_kinematics(double theta1, double theta2, double theta3,
                              double theta4, double theta5, double theta6) {
    // Parte A: posicion del sistema 4
    double phi2  = theta2;
    double phi23 = theta2 + theta3;

    double r4 = L2*cos(phi2) + LD4*cos(phi23);
    double z4 = L1 + L2*sin(phi2) + LD4*sin(phi23);
    double x4 = r4*cos(theta1);
    double y4 = r4*sin(theta1);

    // Orientacion: R05 = R1*R2*R3*R4*R5*R6 (misma tabla DH que el MATLAB)
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

    double R11 = R06[0][0], R21 = R06[1][0], R31 = R06[2][0];
    double R32 = R06[2][1], R33 = R06[2][2];
    double R13 = R06[0][2], R23 = R06[1][2];  // columna 3 -- direccion de Z6 en el mundo

    // Parte B: se le suma el aporte de la muñeca (Ld6) en la direccion de Z6
    double x = x4 + LD6*R13;
    double y = y4 + LD6*R23;
    double z = z4 + LD6*R33;

    double mag   = sqrt(R11*R11 + R21*R21); // = |cos(pitch)|
    double pitch = atan2(-R31, mag);
    double yaw, roll;

    if (mag < EPS_SINGULARIDAD) {
        // Gimbal lock: solo yaw+roll esta definido, se estandariza igual que
        // en el brazo de 5R (acordado con el profesor).
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
// Reloj a 216 MHz (identico a FK_5R_Geometrico_STM32)
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
// Contador de ciclos DWT->CYCCNT
// ---------------------------------------------------------------------------
void DWT_Init(void) {
    CoreDebug->DEMCR |= (1<<24);           // TRCENA
    DWT->CYCCNT = 0;
    DWT->CTRL |= (1<<0);                   // CYCCNTENA
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

    char buf[180];

    // Medicion "en frio" -- una sola ejecucion, comparable directo con los
    // ~152-160 ciclos de la FPGA (version con multiplicador reutilizado)
    uint32_t c0 = DWT->CYCCNT;
    FK_Result r = forward_kinematics(t1, t2, t3, t4, t5, t6);
    uint32_t ciclos_frio = DWT->CYCCNT - c0;

    // Medicion en estado estable -- promedio de N_REPS ejecuciones seguidas
    c0 = DWT->CYCCNT;
    for (int i = 0; i < N_REPS; i++) {
        r = forward_kinematics(t1, t2, t3, t4, t5, t6);
    }
    uint32_t ciclos_prom = (DWT->CYCCNT - c0) / N_REPS;

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
        "  ciclos (1 ejecucion, en frio)    = %lu  (%.3f us @ 216MHz)\r\n"
        "  ciclos (promedio %d ejecuciones) = %lu  (%.3f us @ 216MHz)\r\n",
        (unsigned long)ciclos_frio, ciclos_frio / 216.0,
        N_REPS, (unsigned long)ciclos_prom, ciclos_prom / 216.0);
    USART3_SendString(buf);
}

void run_all_cases(void) {
    // Casos de prueba -- los mismos que se corrieron en ModelSim (leccion 12)
    run_test_case("Caso 1 [extendido] q=0",                0,  0,   0,  0,  0,   0);
    run_test_case("Caso 2 [SINGULARIDAD del 6R] th5=90 th6=90", 0, 0, 0, 0, 90, 90);
    run_test_case("Caso 3 [generico]",                     30, 20, -15, 45, 60, -70);
}

// ---------------------------------------------------------------------------
// Boton de usuario (PC13) -- repite las 3 pruebas cada vez que se presiona.
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

    DWT_Init();
    USART3_Init();
    Boton_Init();

    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);

    USART3_SendString("\r\n\r\n=== Cinematica Directa 6 GDL real -- STM32F767ZI @ 216MHz ===\r\n");
    USART3_SendString("Presiona el boton de usuario (B1) para repetir las 3 pruebas.\r\n");
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
