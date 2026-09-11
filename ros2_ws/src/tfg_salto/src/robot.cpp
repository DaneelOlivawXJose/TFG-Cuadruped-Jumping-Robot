#include "tfg_salto/robot.hpp"


Robot::Robot() : Node("nodo_robot")
{
    // --- Inicializar patas: {canal_s1, canal_s2, pwm_h_s1, pwm_d_s1, pwm_h_s2, pwm_d_s2}, desfase, inversor_x ---
    misPatas_[0] = { {1, 8, 299, 584, 941, 671}, 0.75f,  1.0f };
    misPatas_[1] = { {3, 2, 715, 992, 890, 634}, 0.50f,  1.0f };
    misPatas_[2] = { {7, 6, 183, 466, 907, 630}, 0.25f, -1.0f };
    misPatas_[3] = { {4, 5, 650, 310, 475, 754}, 0.00f,  1.0f };

    tiempo_inicio_ = this->now();

    // --- Publishers ---
    pub_poses_      = this->create_publisher<std_msgs::msg::Int32MultiArray>("servo_poses",    10);
    pub_meta_salto_ = this->create_publisher<std_msgs::msg::Int32>          ("cmd_meta_salto", 10);

    // --- Subscriptores ---
    sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Robot::cmd_vel_callback, this, std::placeholders::_1));

    sub_estado_ = this->create_subscription<std_msgs::msg::Int32>(
        "cmd_estado", 10, std::bind(&Robot::cmd_estado_callback, this, std::placeholders::_1));

    sub_salto_ = this->create_subscription<std_msgs::msg::Float64>(
        "cmd_salto", 10, std::bind(&Robot::cmd_salto_callback, this, std::placeholders::_1));

    sub_salto_listo_ = this->create_subscription<std_msgs::msg::Int32>(
        "estado_salto_listo", 10, std::bind(&Robot::salto_listo_callback, this, std::placeholders::_1));

    // --- Timer principal: 50 Hz (20 ms) ---
    timer_control_ = this->create_wall_timer(
        20ms, std::bind(&Robot::bucle_control_callback, this));

    RCLCPP_INFO(this->get_logger(), "Nodo Robot iniciado.");
}

Robot::~Robot()
{
    RCLCPP_INFO(this->get_logger(), "Apagando nodo del robot.");
}

// ============================================================
// BUCLE PRINCIPAL (50 Hz)
// ============================================================

void Robot::bucle_control_callback()
{
    // Al cambiar de estado, se hace una pausa de 1 s en REPOSO antes de continuar
    if (cambiado_) {
        estado_previo_ = estado_actual_;
        estado_actual_ = REPOSO;
        ciclos_espera_ = CICLOS_PAUSA_ESTADO;
        cambiado_ = false;
    }

    switch (estado_actual_) {
        case ANDAR:           estado_andar();           break;
        case AGACHADO:        estado_agachado();        break;
        case REPOSO:          estado_reposo();          break;
        case PREPARAR_SALTO:  estado_preparar_salto();  break;
        case CARGAR_MUELLE:   estado_cargar_muelle();   break;
        case LIBERAR_MUELLE:  estado_liberar_muelle();  break;
        case RECUPERAR_REPOSO: estado_recuperar_reposo(); break;
    }
}

// ============================================================
// LÓGICA DE ESTADOS
// ============================================================

void Robot::estado_andar()
{
    // Tiempo global normalizado [0, 1) dentro del ciclo de marcha
    auto tiempo_actual = this->now() - tiempo_inicio_;
    double ms = tiempo_actual.nanoseconds() / 1e6;
    float t_global = static_cast<float>(fmod(ms, CICLO_MS)) / CICLO_MS;

    std_msgs::msg::Int32MultiArray msg;
    msg.data.resize(10);

    for (int i = 0; i < 4; i++) {
        // Cada pata tiene su desfase de fase en el ciclo
        float t_local = fmod(t_global + misPatas_[i].desfase, 1.0f);

        // Las patas delanteras (0,1) restan la compensación y las traseras (2,3) la suman
        float longitud = longitud_paso_ + ((i < 2) ? -compensacion_giro_ : compensacion_giro_);

        float x, y;
        calcularTrayectoriaPaso(t_local, longitud, x, y);

        int s1, s2;
        calcularPataIK(x * misPatas_[i].inversor_x, y, misPatas_[i], s1, s2);
        msg.data[i*2]   = s1;
        msg.data[i*2+1] = s2;
    }

    msg.data[8] = 930;                    // Ratchet en posición de marcha
    msg.data[9] = SERVO_GATILLO_CERRADO;  // Gatillo cerrado durante la marcha
    pub_poses_->publish(msg);
}

void Robot::estado_agachado()
{
    // Baja suavemente el chasis hasta y = 0.02 m (máximo agachado)
    constexpr float Y_AGACHADO = 0.02f;
    constexpr float PASO_BAJA  = 0.0005f;

    if (altura_chasis_ > Y_AGACHADO) {
        altura_chasis_ = std::max(Y_AGACHADO, altura_chasis_ - PASO_BAJA);
    }

    publicarPoses(0.0f, altura_chasis_, 0.0f, 0, SERVO_GATILLO_CERRADO);
}

void Robot::estado_reposo()
{
    constexpr float PASO_ALTURA = 0.0005f;
    constexpr float PASO_THETA  = 0.5f;    // Grados por ciclo al recuperar inclinación

    // Recuperar altura suavemente hacia Y_REPOSO
    if (std::abs(altura_chasis_ - Y_REPOSO) > PASO_ALTURA) {
        altura_chasis_ += (Y_REPOSO > altura_chasis_) ? PASO_ALTURA : -PASO_ALTURA;
    } else {
        altura_chasis_ = Y_REPOSO;
    }

    // Recuperar inclinación suavemente hacia 0°
    theta_actual_ = std::max(0.0f, theta_actual_ - PASO_THETA);

    // Si el estado previo era PREPARAR_SALTO, el muelle sigue cargado: no soltar el gatillo
    int gatillo = (estado_previo_ == PREPARAR_SALTO) ? SERVO_GATILLO_ABIERTO : SERVO_GATILLO_CERRADO;

    publicarPoses(0.0f, altura_chasis_, theta_actual_ * (M_PI / 180.0f), 0, gatillo);

    // Volver al estado anterior tras la pausa
    if (ciclos_espera_ > 0) {
        ciclos_espera_--;
    } else {
        estado_actual_ = estado_previo_;
    }
}

void Robot::estado_cargar_muelle()
{
    // Mantener las patas rígidas en reposo con el gatillo armado
    publicarPoses(0.0f, altura_chasis_, 0.0f, 0, SERVO_GATILLO_ABIERTO);

    // Enviar la orden de tensado al ESP32 una sola vez
    if (!orden_salto_enviada_)
    {
        std_msgs::msg::Int32 msg;
        msg.data = movimiento_servo_;
        pub_meta_salto_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Orden de tensado al ESP32: %d ms", movimiento_servo_);
        orden_salto_enviada_ = true;
    }
}

void Robot::estado_preparar_salto()
{
    // Fase 1: Inclina el chasis y adapta la altura a la longitud física de la pata contraída
    if (!transicion_activa_ && !timer_liberacion_set_)
    {
        RCLCPP_INFO(this->get_logger(), "Ajustando postura a %.1f° y %.3f m de altura...", theta_opt_deg_, y_prep_salto_);
        
        // Aplicamos la altura dinámica (y_prep_salto_) en lugar del Z0 estático
        iniciarTransicion(static_cast<float>(y_prep_salto_), static_cast<float>(theta_opt_deg_), 2.5);
        timer_liberacion_set_ = true; 
    }

    if (actualizarTransicion(altura_chasis_, theta_actual_))
    {
        estado_actual_ = LIBERAR_MUELLE;
        fase_liberacion_ = 0;
        timer_disparo_set_ = false;
    }

    publicarPoses(0.0f, altura_chasis_, theta_actual_ * (M_PI / 180.0f), 0, SERVO_GATILLO_ABIERTO);
}

void Robot::estado_liberar_muelle()
{
    // Fase 2: Liberar el mecanismo (ID 10 a 800)
    if (fase_liberacion_ == 0) {
        RCLCPP_INFO(this->get_logger(), "¡SALTO! Liberando mecanismo ID 10 a 800...");
        
        // gatillo_s10 (índice 9 en tu array) lo enviamos a 800
        publicarPoses(0.0f, altura_chasis_, theta_actual_ * (M_PI / 180.0f), 0, 800);
        
        tiempo_fase_ = this->now();
        fase_liberacion_ = 1;
    } 
    else {
        // Mantén la publicación del gatillo suelto para asegurarnos de que el servo llega
        publicarPoses(0.0f, altura_chasis_, theta_actual_ * (M_PI / 180.0f), 0, 800);
        
        // Esperamos medio segundo mecánico para que el gatillo realmente se suelte
        if ((this->now() - tiempo_fase_).seconds() >= 0.5) {
            RCLCPP_INFO(this->get_logger(), "Recuperando posición inicial...");
            estado_actual_ = RECUPERAR_REPOSO;
            transicion_activa_ = false; // Reseteamos la bandera para el siguiente estado
        }
    }
}

void Robot::estado_recuperar_reposo()
{
    // Fase 3: Vuelve poco a poco a la posición de reposo durante 3 segundos
    if (!transicion_activa_) {
        iniciarTransicion(Y_REPOSO, 0.0f, 3.0);
    }

    // La función bloquea lógicamente el estado durante 3s
    if (actualizarTransicion(altura_chasis_, theta_actual_)) {
        RCLCPP_INFO(this->get_logger(), "Posición de reposo alcanzada. Secuencia terminada.");
        estado_actual_ = REPOSO;
        cambiado_ = true; // Activa la pausa post-estado por seguridad si la deseas
    }

    // Publicamos las poses interpoladas
    publicarPoses(0.0f, altura_chasis_, theta_actual_ * (M_PI / 180.0f), 0, SERVO_GATILLO_CERRADO);
}

// ============================================================
// CALLBACKS ROS
// ============================================================

void Robot::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    bool parado = (msg->linear.x == 0.0 && msg->angular.z == 0.0);

    if (parado) {
        // Solo volver a REPOSO si no estamos en alguna fase del salto
        bool en_salto = (estado_actual_ == AGACHADO     ||
                         estado_actual_ == PREPARAR_SALTO ||
                         estado_actual_ == CARGAR_MUELLE  ||
                         estado_actual_ == LIBERAR_MUELLE);
        if (!en_salto && estado_actual_ != REPOSO) {
            estado_actual_ = REPOSO;
        }
        longitud_paso_     = 0.0f;
        compensacion_giro_ = 0.0f;
    } else {
        estado_actual_     = ANDAR;
        longitud_paso_     = static_cast<float>(msg->linear.x)  * 0.03f;
        compensacion_giro_ = static_cast<float>(msg->angular.z) * 0.002f;
    }
}

void Robot::cmd_estado_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    switch (msg->data) {
        case 0:
            RCLCPP_INFO(this->get_logger(), "→ ANDAR");
            estado_actual_ = ANDAR;
            cambiado_ = true;
            break;
        case 1:
            RCLCPP_INFO(this->get_logger(), "→ AGACHADO");
            estado_actual_ = AGACHADO;
            cambiado_ = true;
            break;
        case 2:
            RCLCPP_INFO(this->get_logger(), "→ CARGAR_MUELLE");
            iniciarSecuenciaSalto(h_objetivo_);
            break;
        case 3:
            RCLCPP_INFO(this->get_logger(), "→ REPOSO");
            estado_actual_ = REPOSO;
            cambiado_ = true;
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "cmd_estado desconocido: %d", msg->data);
    }
}

void Robot::cmd_salto_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    iniciarSecuenciaSalto(msg->data);
}

void Robot::salto_listo_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (msg->data == 1) {
        RCLCPP_INFO(this->get_logger(), "¡Muelle cargado! Pasando a PREPARAR_SALTO.");
        estado_actual_ = PREPARAR_SALTO;
        cambiado_ = true;
    }
}

// ============================================================
// HELPER: Iniciar secuencia de salto
// ============================================================

void Robot::iniciarSecuenciaSalto(double h)
{
    h_objetivo_          = h;
    theta_actual_        = 0.0f;
    altura_chasis_       = Y_REPOSO;
    
    // Reset de todas las banderas para garantizar que un segundo salto funcione
    orden_salto_enviada_  = false;
    transicion_activa_    = false;
    timer_liberacion_set_ = false;
    timer_disparo_set_    = false;
    fase_liberacion_      = 0;
    
    estado_actual_       = CARGAR_MUELLE;
    cambiado_            = true;
    calcularParametrosSalto(h);
}

// ============================================================
// CINEMÁTICA INVERSA Y TRAYECTORIAS
// ============================================================

void Robot::calcularPataIK(float x, float y, const Pata &p, int &posS1, int &posS2) const
{
    // Ángulos auxiliares en el triángulo formado por los eslabones
    float alfa1 = atan2(y, x + D_PATA / 2.0f);
    float alfa2 = atan2(y, x - D_PATA / 2.0f);

    float a1 = std::sqrt((x + D_PATA/2.0f)*(x + D_PATA/2.0f) + y*y);
    float a2 = std::sqrt((x - D_PATA/2.0f)*(x - D_PATA/2.0f) + y*y);

    float b1 = (L1*L1 + a1*a1 - L3*L3) / (2.0f * L1 * a1);
    float b2 = (L2*L2 + a2*a2 - L4*L4) / (2.0f * L2 * a2);

    // Comprobar límite físico (argumento de acos fuera de [-1, 1])
    if (std::abs(b1) > 1.0f || std::abs(b2) > 1.0f) return;

    float theta1_deg = (alfa1 + std::acos(b1)) * (180.0f / M_PI);
    float theta2_deg = (alfa2 - std::acos(b2)) * (180.0f / M_PI);

    posS1 = gradosAPasos(p, 1, theta1_deg);
    posS2 = gradosAPasos(p, 0, theta2_deg);
}

int Robot::gradosAPasos(const Pata &p, int lado, float grados) const
{
    float pasos;
    int minSeguro, maxSeguro;

    if (lado == 0) {  // Servo 2 (inferior)
        float factor = (p.config.pwm_down_s2 - p.config.pwm_horiz_s2) / 90.0f;
        pasos    = p.config.pwm_horiz_s2 + (grados - 0.0f) * factor;
        minSeguro = std::min(p.config.pwm_horiz_s2, p.config.pwm_down_s2 - 200);
        maxSeguro = std::max(p.config.pwm_horiz_s2, p.config.pwm_down_s2 + 200);
    } else {           // Servo 1 (superior)
        float factor = (p.config.pwm_down_s1 - p.config.pwm_horiz_s1) / -90.0f;
        pasos    = p.config.pwm_horiz_s1 + (grados - 180.0f) * factor;
        minSeguro = std::min(p.config.pwm_horiz_s1, p.config.pwm_down_s1 - 200);
        maxSeguro = std::max(p.config.pwm_horiz_s1, p.config.pwm_down_s1 + 200);
    }

    return std::clamp(static_cast<int>(pasos), minSeguro, maxSeguro);
}

void Robot::calcularTrayectoriaPaso(float t_local, float longitud,
                                    float &out_x, float &out_y) const
{
    if (t_local < FRACCION_AIRE) {
        // Fase de vuelo: arco senoidal de -L/2 a +L/2
        float progreso = t_local / FRACCION_AIRE;
        out_x = -longitud / 2.0f + longitud * progreso;
        out_y = ALTURA_SUELO - ALTURA_LEVANT * std::sin(progreso * M_PI);
    } else {
        // Fase de apoyo: deslizamiento rectilíneo de +L/2 a -L/2
        float progreso = (t_local - FRACCION_AIRE) / (1.0f - FRACCION_AIRE);
        out_x = longitud / 2.0f - longitud * progreso;
        out_y = ALTURA_SUELO;
    }
}

// ============================================================
// HELPER: Publicar poses con inclinación opcional
// ============================================================

/// Calcula la IK para las 4 patas y publica el mensaje de poses.
/// @param theta_rad  Ángulo de inclinación del chasis en radianes (0 = plano).
///                   Las patas 0 y 2 (delanteras) bajan según d_pata × tan(θ).
void Robot::publicarPoses(float x_objetivo, float y_base,
                          float theta_rad,
                          int gatillo_s9, int gatillo_s10)
{
    std_msgs::msg::Int32MultiArray msg;
    msg.data.resize(10);

    for (int i = 0; i < 4; i++) {
        // Las patas delanteras (índices 0 y 2) se ajustan según la inclinación
        float y_ik = (i == 0 || i == 2)
                     ? y_base - D_PATA * std::tan(theta_rad)
                     : y_base;

        int s1, s2;
        calcularPataIK(x_objetivo * misPatas_[i].inversor_x, y_ik, misPatas_[i], s1, s2);
        msg.data[i*2]   = s1;
        msg.data[i*2+1] = s2;
    }

    msg.data[8] = gatillo_s9;
    msg.data[9] = gatillo_s10;
    pub_poses_->publish(msg);
}

// ============================================================
// GESTIÓN DE TRANSICIONES TEMPORALES
// ============================================================

void Robot::iniciarTransicion(float y_fin, float theta_fin, double duracion_segundos)
{
    y_inicial_ = altura_chasis_;
    theta_inicial_ = theta_actual_;
    y_final_ = y_fin;
    theta_final_ = theta_fin;
    duracion_transicion_ = duracion_segundos;
    tiempo_inicio_transicion_ = this->now();
    transicion_activa_ = true;
}

bool Robot::actualizarTransicion(float &out_y, float &out_theta)
{
    if (!transicion_activa_) return true;

    double transcurrido = (this->now() - tiempo_inicio_transicion_).seconds();
    
    if (transcurrido >= duracion_transicion_) {
        // Hemos terminado el tiempo asignado
        out_y = y_final_;
        out_theta = theta_final_;
        transicion_activa_ = false;
        return true;
    }

    // Calculamos el progreso entre 0.0 y 1.0
    float progreso = static_cast<float>(transcurrido / duracion_transicion_);
    
    // Interpolación lineal
    out_y = y_inicial_ + progreso * (y_final_ - y_inicial_);
    out_theta = theta_inicial_ + progreso * (theta_final_ - theta_inicial_);
    
    return false; // Aún estamos en transición
}

// ============================================================
// CÁLCULO DE PARÁMETROS DE SALTO
// ============================================================

void Robot::calcularParametrosSalto(double h_objetivo)
{
    // Rango de búsqueda del ángulo de lanzamiento
    constexpr int    RESOLUCION    = 1000;
    const double theta_min = std::atan(1.0 / MU_FRICCION);
    const double theta_max = (M_PI / 2.0) - 0.01;
    const double paso_theta = (theta_max - theta_min) / (RESOLUCION - 1);

    double v0_sq_min   = std::numeric_limits<double>::infinity();
    double theta_opt   = 0.0;

    // Búsqueda del ángulo que minimiza la velocidad de despegue necesaria.
    // Con d_min == d_max la búsqueda en d es trivial; se mantiene el doble
    // bucle por compatibilidad con futuras variaciones de la distancia.
    for (int i = 0; i < RESOLUCION; ++i) {
        double theta     = theta_min + i * paso_theta;
        double cos2      = std::cos(theta) * std::cos(theta);
        double tan_theta = std::tan(theta);
        
        // El Z0 de despegue depende de la inclinación del robot con la pata extendida
        double z0_takeoff = LONGITUD_PATA_SALTO * std::sin(theta);
        double denom      = z0_takeoff + DIST_SALTO * tan_theta - h_objetivo;

        if (denom <= 0.0) continue;  // La trayectoria no alcanza la altura

        double v0_sq = (DIST_SALTO * DIST_SALTO * G) / (2.0 * cos2 * denom);
        if (v0_sq < v0_sq_min) {
            v0_sq_min = v0_sq;
            theta_opt = theta;
        }
    }

    if (std::isinf(v0_sq_min)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Ninguna trayectoria alcanza h = %.2f m. Salto cancelado.", h_objetivo);
        return;
    }

    double v0 = std::sqrt(v0_sq_min);

    // Convertir a ángulo de inclinación del chasis (complementario al de lanzamiento)
    theta_opt_deg_ = 90.0 - theta_opt * (180.0 / M_PI);

    // Compresión necesaria del muelle
    double m_g_sin     = MASA_ROBOT * G * std::sin(theta_opt);
    double delta_x     = (m_g_sin + std::sqrt(m_g_sin*m_g_sin + 2*EFICIENCIA*(K_MUELLE_1 + K_MUELLE_2) * MASA_ROBOT * v0_sq_min)) / (2*EFICIENCIA*(K_MUELLE_1 + K_MUELLE_2));

    y_prep_salto_ = (LONGITUD_PATA_SALTO - delta_x) * std::sin(theta_opt);
    if (y_prep_salto_ < 0.02) y_prep_salto_ = 0.02;

    // --- Resultados de diagnóstico ---
    RCLCPP_INFO(this->get_logger(),
        "SALTO: v0=%.2f m/s | incl=%.1f° | dist=%.2f m | Δx_muelle=%.4f m | Y_prep=%.3f m",
        v0, theta_opt_deg_, DIST_SALTO, delta_x, y_prep_salto_);

    if ((delta_x*100) >= 2.4)
    {
        RCLCPP_WARN(this->get_logger(), "Compresión requerida (%.1f cm) excede el muelle. Salto NO viable.", delta_x * 100.0);
        movimiento_servo_  =0.0;
    } else {
        // Tiempo de activación del servo
        double t_servo_s = (Z2*Z4*Z_CARRETE*1023*delta_x) / (MG_0_1023 * Z1*Z3*Z5 * W_MAX_SERVO * R_CARRETE);
        movimiento_servo_ = t_servo_s*1000;
    }

    
}

// ============================================================
// MAIN
// ============================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
    return 0;
}

// ============================================================
// COMANDOS ÚTILES (referencia rápida)
// ============================================================
// ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
// ros2 run tfg_salto tfg_robot
// ros2 topic pub -1 /cmd_salto std_msgs/msg/Float64 "{data: 0.04}"
// python3 teleop_teclado.py
//
// Servo gatillo (servo 9/10 en el array):
//   800  → gatillo cerrado (reposo / marcha)
//   920  → gatillo libre   (disparo)
//   1023 → muelle cargado  (armado)
