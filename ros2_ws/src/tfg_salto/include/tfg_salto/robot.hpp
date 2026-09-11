#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

// ============================================================
// CONSTANTES CINEMÁTICAS DE LA PATA (metros)
// ============================================================
constexpr float D_PATA  = 0.022f;  // Separación entre los dos pivotes de la pata
constexpr float L1      = 0.020f;  // Longitud del eslabón superior izquierdo
constexpr float L2      = L1;      // Longitud del eslabón superior derecho (simétrico)
constexpr float L3      = 0.030f;  // Longitud del eslabón inferior izquierdo
constexpr float L4      = L3;      // Longitud del eslabón inferior derecho (simétrico)
constexpr double Z0     = 0.029;   // Altura de despegue del chasis (m)

// ============================================================
// CONSTANTES DE MARCHA
// ============================================================
constexpr float ALTURA_SUELO       = 0.040f;  // Altura nominal del chasis al suelo (m)
constexpr float ALTURA_LEVANT      = 0.015f;  // Altura que levanta la pata en el aire (m)
constexpr float Y_REPOSO           = 0.046f;  // Altura del chasis en posición de reposo (m)
constexpr int   CICLO_MS           = 3000;    // Duración de un ciclo de marcha completo (ms)
constexpr float FRACCION_AIRE      = 0.25f;   // Fracción del ciclo que la pata pasa en el aire

// ============================================================
// CONSTANTES DE MARCHA (servo de disparo)
// ============================================================
constexpr int SERVO_GATILLO_CERRADO = 800;    // PWM con el gatillo retenido
constexpr int SERVO_GATILLO_ABIERTO = 1023;   // PWM con el muelle cargado/gatillo armado
constexpr int SERVO_GATILLO_LIBRE   = 920;    // PWM al soltar (disparar) el muelle

// ============================================================
// CONSTANTES FÍSICAS DEL MECANISMO DE SALTO
// ============================================================
constexpr double MASA_ROBOT   = 0.370;   // Masa total del robot (kg)
constexpr double G            = 9.81;   // Gravedad (m/s²)
constexpr double K_MUELLE_1     = 170.0;  // Constante elástica del muelle (N/m)
constexpr double K_MUELLE_2     = 200.0;  // Constante elástica del muelle (N/m)
constexpr double EFICIENCIA   = 1.0;   // Eficiencia mecánica del muelle
constexpr double MU_FRICCION  = 0.90;   // Coeficiente de fricción estática
constexpr double DIST_SALTO   = 0.07;   // Distancia horizontal de despegue al obstáculo (m)
constexpr double ALT_OBSTAC_DEFAULT = 0.035; // Altura de obstáculo por defecto (m)
constexpr double LONGITUD_PATA_SALTO = 0.042;
constexpr int Z1 = 10;
constexpr int Z2 = 30;
constexpr int Z3 = 10;
constexpr int Z4 = 30;
constexpr int Z5 = 10;
constexpr int Z_CARRETE = 20;
constexpr int MG_0_1023 = 600;
constexpr double W_MAX_SERVO = 10.4719755;
constexpr double R_CARRETE = 0.004;


// Ciclos de pausa (a 50 Hz) al cambiar de estado
constexpr int CICLOS_PAUSA_ESTADO = 50; // 50 × 20 ms = 1 segundo

// ============================================================
// ESTRUCTURAS DE DATOS
// ============================================================

/// Parámetros hardware de calibración de una pata (canales y posiciones PWM)
struct ConfigHardware {
    int s1, s2;
    int pwm_horiz_s1, pwm_down_s1;
    int pwm_horiz_s2, pwm_down_s2;
};

/// Representa una pata con su configuración hardware y parámetros de marcha
struct Pata {
    ConfigHardware config;
    float desfase;    // Desfase de fase en el ciclo de marcha [0, 1)
    float inversor_x; // +1.0 o -1.0 según la orientación lateral de la pata
};

/// Estados de la máquina de estados del robot
enum Estado {
    ANDAR,
    AGACHADO,
    REPOSO,
    PREPARAR_SALTO,
    CARGAR_MUELLE,
    LIBERAR_MUELLE,
    RECUPERAR_REPOSO
};

// ============================================================
// CLASE ROBOT
// ============================================================
class Robot : public rclcpp::Node
{
public:
    Robot();
    ~Robot();

private:
    // --- Patas ---
    Pata misPatas_[4];

    // --- Máquina de estados ---
    Estado estado_actual_  = REPOSO;
    Estado estado_previo_  = REPOSO;
    bool   cambiado_       = false;  // true cuando se acaba de cambiar de estado
    int    ciclos_espera_  = 0;

    // --- Variables de locomoción ---
    float longitud_paso_     = 0.0f;
    float compensacion_giro_ = 0.0f;
    float altura_chasis_     = Y_REPOSO;
    rclcpp::Time tiempo_inicio_;   // Referencia temporal para la marcha

    // --- Variables de salto ---
    double h_objetivo_           = ALT_OBSTAC_DEFAULT; // Altura del obstáculo (m)
    double theta_opt_deg_        = 0.0;  // Ángulo óptimo de salto (grados)
    float  theta_actual_         = 0.0f; // Ángulo de inclinación actual del chasis (grados)
    int    movimiento_servo_     = 0;    // Duración de tensado del servo (ms)
    bool   orden_salto_enviada_  = false;
    double y_prep_salto_ = 0.0;

    // --- Fase interna del estado LIBERAR_MUELLE ---
    int             fase_liberacion_      = 0;
    bool            timer_liberacion_set_ = false;
    bool            timer_disparo_set_    = false;
    rclcpp::Time    tiempo_fase_;
    
    // --- Control de transiciones temporales (Interpolación) ---
    bool   transicion_activa_ = false;
    double duracion_transicion_ = 0.0;
    rclcpp::Time tiempo_inicio_transicion_;
    float  y_inicial_ = 0.0f, y_final_ = 0.0f;
    float  theta_inicial_ = 0.0f, theta_final_ = 0.0f;

    // Métodos para gestionar las transiciones de tiempo
    void iniciarTransicion(float y_fin, float theta_fin, double duracion_segundos);
    bool actualizarTransicion(float &out_y, float &out_theta);
    
    // Nuevo estado
    void estado_recuperar_reposo();

    // --- ROS: Publishers ---
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_poses_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr           pub_meta_salto_;

    // --- ROS: Subscriptores ---
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  sub_vel_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr       sub_estado_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr     sub_salto_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr       sub_salto_listo_;

    // --- ROS: Timer principal ---
    rclcpp::TimerBase::SharedPtr timer_control_;

    // --------------------------------------------------------
    // Callbacks ROS
    // --------------------------------------------------------
    void bucle_control_callback();
    void cmd_vel_callback    (const geometry_msgs::msg::Twist::SharedPtr msg);
    void cmd_estado_callback (const std_msgs::msg::Int32::SharedPtr      msg);
    void cmd_salto_callback  (const std_msgs::msg::Float64::SharedPtr    msg);
    void salto_listo_callback(const std_msgs::msg::Int32::SharedPtr      msg);

    // --------------------------------------------------------
    // Lógica de estados
    // --------------------------------------------------------
    void estado_andar();
    void estado_agachado();
    void estado_reposo();
    void estado_cargar_muelle();
    void estado_preparar_salto();
    void estado_liberar_muelle();

    // --------------------------------------------------------
    // Helpers de cinemática y trayectoria
    // --------------------------------------------------------

    /// Calcula las posiciones PWM de los dos servos de una pata mediante IK
    void calcularPataIK(float x, float y, const Pata &p, int &posS1, int &posS2) const;

    /// Rellena el array de 10 poses con las 4 patas en la posición IK(x, y_i)
    /// permitiendo offsets de inclinación por pata. gatillo_s9 y gatillo_s10
    /// se escriben en las posiciones [8] y [9] del array.
    void publicarPoses(float x_objetivo, float y_base,
                       float theta_rad,
                       int gatillo_s9, int gatillo_s10);

    /// Convierte ángulo en grados a posición PWM para un servo de la pata
    int gradosAPasos(const Pata &p, int lado, float grados) const;

    /// Devuelve la posición (x, y) de la pata en el instante t_local ∈ [0,1)
    void calcularTrayectoriaPaso(float t_local, float longitud,
                                 float &out_x, float &out_y) const;

    /// Calcula los parámetros óptimos del salto para superar h_objetivo
    void calcularParametrosSalto(double h_objetivo);

    /// Prepara las variables de salto y transiciona a CARGAR_MUELLE
    void iniciarSecuenciaSalto(double h);
};

#endif // ROBOT_HPP