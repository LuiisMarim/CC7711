/*
 * Adaptado: find_light_box -> TesteProjeto.wbt
 * Objetivo: Selecionar dinamicamente a caixa de MENOR MASSA e ir até ela
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <webots/lidar.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP         32

// Velocidades
#define V_FWD             3.0
#define V_TURN_GAIN       2.0
#define V_SPIN            2.0
#define V_BACK           -2.0

// Evasão 
#define OBSTACLE_THRESHOLD 80.0    
#define CRITICAL_THRESHOLD 150.0  
#define SIDE_THRESHOLD     60.0    

// Detecção da caixa
#define DETECT_DIST       0.1

// Limites para varredura de DEFs de caixas
#define MAX_BOXES         64
#define BOX_DEF_PREFIX    "CAIXA"

// Estrutura para armazenar informações da caixa
typedef struct {
    WbNodeRef node;
    char name[32];
    double mass;
    int has_explicit_mass;
} BoxInfo;

// Função para obter massa
static double get_solid_mass(WbNodeRef solid, int *has_explicit_mass) {
    if (!solid) return INFINITY;
    
    *has_explicit_mass = 0;
    
    WbFieldRef mass_field = wb_supervisor_node_get_field(solid, "mass");
    if (mass_field) {
        double mass = wb_supervisor_field_get_sf_float(mass_field);
        if (mass > 0.0) {
            *has_explicit_mass = 1;
            return mass;
        }
    }
    
    WbFieldRef size_field = wb_supervisor_node_get_field(solid, "size");
    if (size_field) {
        const double *size = wb_supervisor_field_get_sf_vec3f(size_field);
        double volume = size[0] * size[1] * size[2];
        return volume;
    }
    
    return INFINITY;
}

// Varre DEFs CAIXA00..CAIXA63, retorna a de menor massa
static WbNodeRef pick_lightest_box(char *out_def, size_t out_def_size) {
    BoxInfo boxes[MAX_BOXES];
    int box_count = 0;
    int has_explicit_mass = 0;
    
    printf("=== PROCURANDO CAIXA MAIS LEVE ===\n");
    
    for (int i = 1; i <= MAX_BOXES; ++i) {
        char name[32];
        snprintf(name, sizeof(name), BOX_DEF_PREFIX "%02d", i);
        WbNodeRef node = wb_supervisor_node_get_from_def(name);
        if (!node) continue;
        
        double mass = get_solid_mass(node, &has_explicit_mass);
        
        if (mass > 0.0 && mass != INFINITY) {
            strcpy(boxes[box_count].name, name);
            boxes[box_count].node = node;
            boxes[box_count].mass = mass;
            boxes[box_count].has_explicit_mass = has_explicit_mass;
            box_count++;
            
            printf("Caixa %s: %s = %.3f\n", name, 
                   has_explicit_mass ? "MASSA" : "volume", mass);
        }
    }
    
    if (box_count == 0) {
        printf("[WARN] Nenhuma caixa com massa válida encontrada.\n");
        return NULL;
    }
    
    printf("\n=== ANALISANDO %d CAIXAS VÁLIDAS ===\n", box_count);
    
    BoxInfo best_explicit = {0};
    int found_explicit = 0;
    
    for (int i = 0; i < box_count; i++) {
        if (boxes[i].has_explicit_mass) {
            if (!found_explicit || boxes[i].mass < best_explicit.mass) {
                best_explicit = boxes[i];
                found_explicit = 1;
                printf("✅ Melhor massa explícita: %s (%.3f)\n", boxes[i].name, boxes[i].mass);
            }
        }
    }
    
    BoxInfo best_volume = {0};
    int found_volume = 0;
    
    for (int i = 0; i < box_count; i++) {
        if (!found_volume || boxes[i].mass < best_volume.mass) {
            best_volume = boxes[i];
            found_volume = 1;
        }
    }
    
    BoxInfo final_best;
    if (found_explicit) {
        final_best = best_explicit;
        printf("\n🎯 SELECIONANDO POR MASSA EXPLÍCITA\n");
    } else if (found_volume) {
        final_best = best_volume;
        printf("\n🎯 SELECIONANDO POR VOLUME\n");
    } else {
        printf("❌ ERRO: Nenhuma caixa válida para seleção.\n");
        return NULL;
    }
    
    printf("🎯 CAIXA MAIS LEVE: %s (%.3f)\n", final_best.name, final_best.mass);
    
    if (out_def && out_def_size) {
        strncpy(out_def, final_best.name, out_def_size);
        out_def[out_def_size - 1] = '\0';
    }
    
    return final_best.node;
}

// Calcula o ângulo entre o robô e o alvo
static double calculate_target_angle(double robot_x, double robot_y, double robot_angle, 
                                    double target_x, double target_y) {
    double dx = target_x - robot_x;
    double dy = target_y - robot_y;
    double target_angle = atan2(dy, dx);
    
    // Normaliza a diferença de ângulo
    double angle_diff = target_angle - robot_angle;
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
    
    return angle_diff;
}

// Obtém a orientação do robô a partir da matriz de rotação
static double get_robot_angle(const double *rotation) {
    return atan2(rotation[3], rotation[0]);
}

typedef enum { BUSCA_DIRETA = 0, ESCAPE = 1, GIRO = 2, DESVIO = 3 } State;

int main() {
    wb_robot_init();

    // Motores
    WbDeviceTag mL = wb_robot_get_device("left wheel motor");
    WbDeviceTag mR = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(mL, INFINITY);
    wb_motor_set_position(mR, INFINITY);

    // Supervisor: nós do robô e caixa-alvo
    WbNodeRef me = wb_supervisor_node_get_self();
    char target_def[32] = {0};
    WbNodeRef box = pick_lightest_box(target_def, sizeof(target_def));
    
    if (!box) {
        printf("❌ ERRO: Nenhuma caixa válida encontrada.\n");
        wb_robot_cleanup();
        return 1;
    }

    // Sensores de proximidade do e-puck - TODOS os 8 sensores
    WbDeviceTag prox[8] = {0};
    const char *prox_names[8] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
    double prox_values[8] = {0};
    
    for (int i = 0; i < 8; ++i) {
        prox[i] = wb_robot_get_device(prox_names[i]);
        if (prox[i]) {
            wb_distance_sensor_enable(prox[i], TIME_STEP);
            printf("Sensor %s habilitado\n", prox_names[i]);
        }
    }
    printf("Todos os sensores de proximidade habilitados.\n");

    // Estado e auxiliares
    State state = BUSCA_DIRETA;
    double escape_start = 0.0;
    int escape_dir = 1;
    double last_dist_log = 0.0;
    int steps_without_progress = 0;
    double last_distance = 1e9;
    int desvio_count = 0;

    // Verifica posição inicial
    const double *initial_robot_pos = wb_supervisor_node_get_position(me);
    const double *box_pos = wb_supervisor_node_get_position(box);
    double initial_dx = box_pos[0] - initial_robot_pos[0];
    double initial_dy = box_pos[1] - initial_robot_pos[1];
    double initial_dist = sqrt(initial_dx*initial_dx + initial_dy*initial_dy);
    
    printf("\n=== POSIÇÃO INICIAL ===\n");
    printf("Robô: (%.3f, %.3f)\n", initial_robot_pos[0], initial_robot_pos[1]);
    printf("Caixa %s: (%.3f, %.3f)\n", target_def, box_pos[0], box_pos[1]);
    printf("Distância inicial: %.3fm\n", initial_dist);

    while (wb_robot_step(TIME_STEP) != -1) {
        // Lê todos os sensores de proximidade
        for (int i = 0; i < 8; i++) {
            if (prox[i]) {
                prox_values[i] = wb_distance_sensor_get_value(prox[i]);
            }
        }
        
        // Posições e orientação
        const double *robot_pos = wb_supervisor_node_get_position(me);
        const double *robot_rot = wb_supervisor_node_get_orientation(me);
        double robot_angle = get_robot_angle(robot_rot);
        
        const double *current_box_pos = wb_supervisor_node_get_position(box);
        double dx = current_box_pos[0] - robot_pos[0];
        double dy = current_box_pos[1] - robot_pos[1];
        double dist = sqrt(dx*dx + dy*dy);
        
        // Calcula ângulo para o alvo
        double angle_to_target = calculate_target_angle(robot_pos[0], robot_pos[1], 
                                                       robot_angle, current_box_pos[0], current_box_pos[1]);

        // Log da distância a cada 0.1m de mudança
        if (fabs(dist - last_dist_log) > 0.1 || state == GIRO) {
            printf("Distância até %s: %.2fm, Estado: %s\n", 
                   target_def, dist,
                   state == BUSCA_DIRETA ? "BUSCA" : 
                   state == ESCAPE ? "ESCAPE" : 
                   state == DESVIO ? "DESVIO" : "GIRO");
            last_dist_log = dist;
        }

        // --- DETECÇÃO DE OBSTÁCULOS MELHORADA ---
        int obstacle_front = 0;
        int obstacle_left = 0;
        int obstacle_right = 0;
        int critical_obstacle = 0;
        
        // Sensores frontais: ps0, ps7 (mais sensíveis)
        if (prox_values[0] > OBSTACLE_THRESHOLD || prox_values[7] > OBSTACLE_THRESHOLD) {
            obstacle_front = 1;
        }
        
        // Sensores frontais-extremos: obstáculo crítico
        if (prox_values[0] > CRITICAL_THRESHOLD || prox_values[7] > CRITICAL_THRESHOLD) {
            critical_obstacle = 1;
        }
        
        // Sensores laterais esquerdos: ps5, ps6
        if (prox_values[5] > SIDE_THRESHOLD || prox_values[6] > SIDE_THRESHOLD) {
            obstacle_left = 1;
        }
        
        // Sensores laterais direitos: ps1, ps2  
        if (prox_values[1] > SIDE_THRESHOLD || prox_values[2] > SIDE_THRESHOLD) {
            obstacle_right = 1;
        }
        
        // Debug dos sensores (apenas quando há obstáculos)
        if (obstacle_front || obstacle_left || obstacle_right) {
            printf("SENSORES: ");
            for (int i = 0; i < 8; i++) {
                printf("%s:%.0f ", prox_names[i], prox_values[i]);
            }
            printf("\n");
        }

        // --- TRANSIÇÕES DE ESTADO MELHORADAS ---
        if (state != GIRO) {
            if (dist < DETECT_DIST) {
                state = GIRO;
                printf("\n🎯🎯🎯 CAIXA '%s' DETECTADA A %.2fm – INICIANDO GIRO! 🎯🎯🎯\n", 
                       target_def, dist);
            } 
            // Obstáculo CRÍTICO - recuar imediatamente
            else if (critical_obstacle && state != ESCAPE) {
                state = ESCAPE;
                escape_start = wb_robot_get_time();
                // Decide direção baseado em qual lado tem menos obstáculos
                escape_dir = (obstacle_left || prox_values[5] > prox_values[2]) ? 1 : -1;
                printf("🚨 OBSTÁCULO CRÍTICO! Recuando e girando para %s\n", 
                       escape_dir > 0 ? "DIREITA" : "ESQUERDA");
            }
            // Obstáculo frontal - desvio suave
            else if (obstacle_front && state == BUSCA_DIRETA) {
                state = DESVIO;
                desvio_count = 0;
                // Escolhe direção baseado em obstáculos laterais
                if (!obstacle_right && obstacle_left) {
                    escape_dir = -1; // Vira para direita (obstáculo na esquerda)
                } else if (!obstacle_left && obstacle_right) {
                    escape_dir = 1;  // Vira para esquerda (obstáculo na direita)
                } else {
                    escape_dir = (rand() % 2) ? 1 : -1; // Aleatório se ambos lados livres
                }
                printf("↪️  Obstáculo frontal - iniciando DESVIO para %s\n",
                       escape_dir > 0 ? "ESQUERDA" : "DIREITA");
            }
            // Fim do desvio (volta para busca)
            else if (state == DESVIO) {
                desvio_count++;
                if (desvio_count > 30 && !obstacle_front) { // ~1 segundo sem obstáculos
                    state = BUSCA_DIRETA;
                    printf("↩️  Fim do desvio - retornando à BUSCA DIRETA\n");
                    last_distance = dist;
                }
            }
            // Fim do escape (volta para busca)
            else if (state == ESCAPE) {
                double now = wb_robot_get_time();
                if (now - escape_start > 2.0) { // Escape mais longo
                    state = BUSCA_DIRETA;
                    printf("↩️  Fim do escape - retornando à BUSCA DIRETA\n");
                    last_distance = dist;
                }
            }
        }

        // --- AÇÕES POR ESTADO MELHORADAS ---
        if (state == BUSCA_DIRETA) {
            // Navegação direta para a caixa com ajuste dinâmico
            double base_speed = V_FWD;
            double turn_gain = V_TURN_GAIN;
            
            // Reduz velocidade baseado em condições
            if (fabs(angle_to_target) > 1.0) {
                base_speed *= 0.5; // Reduz para curvas fechadas
            } else if (dist < 0.4) {
                base_speed *= 0.7; // Reduz quando perto do alvo
            }
            
            // Pequenas correções para obstáculos laterais
            if (obstacle_left && !obstacle_right) {
                angle_to_target -= 0.3; // Desvia para direita
            } else if (obstacle_right && !obstacle_left) {
                angle_to_target += 0.3; // Desvia para esquerda
            }
            
            double left_speed = base_speed - angle_to_target * turn_gain;
            double right_speed = base_speed + angle_to_target * turn_gain;
            
            wb_motor_set_velocity(mL, left_speed);
            wb_motor_set_velocity(mR, right_speed);
            
            // Verifica progresso
            if (dist < last_distance - 0.02) {
                steps_without_progress = 0;
                last_distance = dist;
            } else {
                steps_without_progress++;
                if (steps_without_progress > 100) {
                    printf("🔁 Sem progresso - tentando correção de curso\n");
                    steps_without_progress = 0;
                }
            }

        } else if (state == ESCAPE) {
            // Comportamento de escape agressivo
            double now = wb_robot_get_time();
            double dt = now - escape_start;
            
            if (dt < 0.8) {
                // Recua rapidamente
                wb_motor_set_velocity(mL, V_BACK * 1.2);
                wb_motor_set_velocity(mR, V_BACK * 1.2);
            } else if (dt < 1.8) {
                // Gira agressivamente
                wb_motor_set_velocity(mL, escape_dir * V_FWD * 0.8);
                wb_motor_set_velocity(mR, -escape_dir * V_FWD * 0.8);
            } else {
                // Pequeno avanço após girar
                wb_motor_set_velocity(mL, V_FWD * 0.6);
                wb_motor_set_velocity(mR, V_FWD * 0.6);
            }

        } else if (state == DESVIO) {
            // Desvio suave - mantém movimento para frente enquanto vira
            double turn_strength = 1.5;
            if (obstacle_front) {
                // Se ainda tem obstáculo frontal, vira mais
                turn_strength = 2.5;
            }
            
            wb_motor_set_velocity(mL, V_FWD * 0.7 - escape_dir * turn_strength);
            wb_motor_set_velocity(mR, V_FWD * 0.7 + escape_dir * turn_strength);

        } else { // GIRO
            wb_motor_set_velocity(mL, -V_SPIN);
            wb_motor_set_velocity(mR, V_SPIN);
        }
    }

    wb_robot_cleanup();
    return 0;
}