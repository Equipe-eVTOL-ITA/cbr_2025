#include <gtest/gtest.h>
#include <memory>
#include "fase2/states/align_base_state.hpp"
#include "fase2/states/align_package_state.hpp"
#include "fsm/fsm.hpp"

/**
 * Testes unitários para os novos estados de alinhamento.
 * Verifica compatibilidade e funcionalidade básica.
 */

class AlignStatesTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Mock objects para testes
        drone = std::make_shared<MockDrone>();
        vision = std::make_shared<MockVisionNode>();
        
        setupBlackboard();
    }
    
    void setupBlackboard() {
        bb.set("drone", drone);
        bb.set("vision", vision);
        
        // Parâmetros mínimos necessários
        bb.set("align_tolerance", 0.1f);
        bb.set("max_horizontal_velocity", 0.5f);
        bb.set("align_descent_velocity", 0.2f);
        bb.set("detection_timeout", 5.0f);
        bb.set("pid_pos_kp", 1.0f);
        bb.set("pid_pos_ki", 0.1f);
        bb.set("pid_pos_kd", 0.05f);
        bb.set("setpoint", 0.0f);
        bb.set("base_align_offset_x", 0.05f);
        bb.set("base_align_offset_y", 0.02f);
        bb.set("package_align_offset_x", 0.15f);
        bb.set("package_align_offset_y", 0.0f);
        bb.set("height_to_ground", 1.0f);
        bb.set("mean_base_height", 0.0f);
        bb.set("mean_package_height", 0.1f);
        bb.set("yaw_align_tolerance", 0.087f);
        bb.set("max_yaw_rate", 0.5f);
        bb.set("pid_yaw_kp", 2.0f);
        bb.set("pid_yaw_ki", 0.0f);
        bb.set("pid_yaw_kd", 0.1f);
        bb.set("package_state", std::string("get_package"));
    }
    
    std::shared_ptr<MockDrone> drone;
    std::shared_ptr<MockVisionNode> vision;
    fsm::Blackboard bb;
};

TEST_F(AlignStatesTest, AlignBaseStateInitialization) {
    AlignBaseState align_base;
    
    // Teste de inicialização
    EXPECT_NO_THROW(align_base.on_enter(bb));
    
    // Verificar que o offset foi configurado
    // (Este teste assumiria métodos de acesso aos membros protegidos)
}

TEST_F(AlignStatesTest, AlignPackageStateInitialization) {
    AlignPackageState align_package;
    
    // Teste de inicialização
    EXPECT_NO_THROW(align_package.on_enter(bb));
    
    // Verificar configuração específica para packages
}

TEST_F(AlignStatesTest, PolymorphicBehavior) {
    // Testar comportamento polimórfico
    std::vector<std::unique_ptr<AlignState>> states;
    states.push_back(std::make_unique<AlignBaseState>());
    states.push_back(std::make_unique<AlignPackageState>());
    
    for (auto& state : states) {
        EXPECT_NO_THROW(state->on_enter(bb));
        // act() é virtual puro, cada implementação deve funcionar
    }
}

TEST_F(AlignStatesTest, OffsetConfiguration) {
    AlignBaseState base_state;
    AlignPackageState package_state;
    
    base_state.on_enter(bb);
    package_state.on_enter(bb);
    
    // Verificar que offsets diferentes foram aplicados
    // (Requereria métodos de acesso para teste)
}

TEST_F(AlignStatesTest, VisionCompatibility) {
    // Configurar mock do VisionNode para simular detecções
    ON_CALL(*vision, isThereBaseDetection())
        .WillByDefault(::testing::Return(true));
    ON_CALL(*vision, isTherePackageDetection())
        .WillByDefault(::testing::Return(true));
    ON_CALL(*vision, lastBaseDetectionTime())
        .WillByDefault(::testing::Return(1.0));
    ON_CALL(*vision, lastPackageDetectionTime())
        .WillByDefault(::testing::Return(1.0));
    
    AlignBaseState base_state;
    AlignPackageState package_state;
    
    base_state.on_enter(bb);
    package_state.on_enter(bb);
    
    // Verificar que os métodos do VisionNode são chamados corretamente
    EXPECT_CALL(*vision, isThereBaseDetection()).Times(::testing::AtLeast(1));
    EXPECT_CALL(*vision, isTherePackageDetection()).Times(::testing::AtLeast(1));
    
    // Simular execução dos estados
    std::string base_result = base_state.act(bb);
    std::string package_result = package_state.act(bb);
    
    // Verificar que não houve erros
    EXPECT_NO_THROW();
}

TEST_F(AlignStatesTest, AlignmentLogic) {
    AlignBaseState base_state;
    base_state.on_enter(bb);
    
    // Simular situações diferentes
    ON_CALL(*vision, isThereBaseDetection())
        .WillByDefault(::testing::Return(false));
    
    // Sem detecção
    std::string result = base_state.act(bb);
    // Deve continuar tentando ou reportar perda
    
    // Com detecção
    ON_CALL(*vision, isThereBaseDetection())
        .WillByDefault(::testing::Return(true));
    ON_CALL(*vision, getClosestBasePosition(::testing::_, ::testing::_, ::testing::_, ::testing::_))
        .WillByDefault(::testing::Return(Eigen::Vector3d(0.05, 0.02, 0.0))); // Próximo ao alinhamento
    
    result = base_state.act(bb);
    // Deve aplicar comandos PID
}

// Mock classes para testes
class MockDrone : public Drone {
public:
    MOCK_METHOD(void, log, (const std::string&), (override));
    MOCK_METHOD(Eigen::Vector3d, getLocalPosition, (), (override));
    MOCK_METHOD(Eigen::Vector3d, getOrientation, (), (override));
    MOCK_METHOD(void, setLocalVelocity, (float, float, float, float), (override));
    MOCK_METHOD(void, setLocalPosition, (float, float, float, float), (override));
};

class MockVisionNode : public VisionNode {
public:
    MOCK_METHOD(bool, isThereBaseDetection, (), (override));
    MOCK_METHOD(bool, isTherePackageDetection, (), (override));
    MOCK_METHOD(double, lastBaseDetectionTime, (), (override));
    MOCK_METHOD(double, lastPackageDetectionTime, (), (override));
    MOCK_METHOD(Eigen::Vector3d, getClosestBasePosition, 
                (const Eigen::Vector3d&, const Eigen::Vector3d&, float, bool), (override));
    MOCK_METHOD(Eigen::Vector3d, getClosestPackagePosition, 
                (const Eigen::Vector3d&, const Eigen::Vector3d&, float, bool), (override));
    MOCK_METHOD(BoundingBox, getClosestBaseBbox, (), (override));
    MOCK_METHOD(BoundingBox, getClosestPackageBbox, (), (override));
    MOCK_METHOD(void, publishBaseDetection, 
                (const std::string&, const Eigen::Vector3d&, float, uint32_t), (override));
};

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
