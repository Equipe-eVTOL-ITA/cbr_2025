#include "align_state.hpp"
#include "fase4/aux/movement.hpp"

class AlignWindowState : public AlignState {
private:
    bool aligned;
    int janela_counter;

public:
    AlignWindowState() : AlignState() {}

    void on_enter(fsm::Blackboard &bb){
        AlignState::on_enter(bb);
        this->aligned = false;
        this->janela_counter = static_cast<int>(*bb.get<float>("janela_counter"));
    }
    
    std::string act(fsm::Blackboard &bb) override {
        AlignState::act(bb); // já atualiza o pos e o orientation

        // verifica se há detecção de janela
        if (!this->vision->isThereJanelaDetection()) {
            if (this->print_counter % 50 == 0) {
                this->drone->log("ALIGN_WINDOW: Waiting for window detection...");
            }

            return "NO WINDOW DETECTED";
        }

        Janela janela = this->vision->getJanela();
        
        // log da janela
        if (this->print_counter % 50 == 0) {
            this->drone->log("ALIGN_WINDOW: Window detected at x=" + 
                           std::to_string(janela.center_x) + ", y=" + 
                           std::to_string(janela.center_y));
        }

        // Calcula erro de posição (janela no centro = erro zero)
        // Valores normalizados [-1, +1] onde 0 = centro da imagem
        double error_x = janela.center_x;  // -1 (esquerda) a +1 (direita)
        double error_y = janela.center_y;  // -1 (cima) a +1 (baixo)
        
        // pid com valores normalizados
        this->x_pid.setSetpoint(0.0); // centro da imagem
        this->y_pid.setSetpoint(0.0); // centro da imagem
        double control_y = this->x_pid.compute(error_x);  // Janela direita → drone direita (+Y)
        double control_z = this->y_pid.compute(error_y);  // Janela baixo → drone baixo (+Z)
        
        // Aplica comando de velocidade usando movimento local
        // X=frente, Y=direita, Z=baixo
        move_local_by_speed(this->drone, 0.0f, static_cast<float>(control_y), static_cast<float>(control_z));
        
        double distance_error = std::sqrt(error_x * error_x + error_y * error_y);
        
        if (distance_error < this->align_tolerance) {
            if (!this->aligned) {
                this->drone->log("ALIGN_WINDOW: Window aligned! Distance error: " + 
                               std::to_string(distance_error));
                this->aligned = true;
            }

            // para o drone
            move_local_by_speed(this->drone, 0.0f, 0.0f, 0.0f);
            return "ALIGNED";
        } else {
            this->aligned = false;
        }
        
        return "";
    }

protected:
    std::string getAlignmentType() const override {
        return "WINDOW";
    }
    
    void configureOffset(fsm::Blackboard &bb) override {
        // Offset específico para alinhamento com janelas
        this->offset.x() = *bb.get<float>("window_align_offset_x");
        this->offset.y() = *bb.get<float>("window_align_offset_y");
    }
};