#define NORMAL 1
#define BUMPLESS 2
#define TWO_DOF 3
#define NORMAL_WITH_FF 3
#define BUMPLESS_WITH_FF 4

struct PIDSettings{
  float kp, ki, kd, ts, tdel, vmax, vmin, a1, a2; //a1, a2はFF制御用
  uint8_t mode;
};

class PIDCalculator{
public:
  float kp, ki, kd, ts, tdel, a1, a2; //a1, a2はフィードフォワード制御用の定数
  PIDCalculator(PIDSettings);
  float calcValue(float, float); //誤差から出力を計算
  float getP();
  float getI();
  float getD();
  
private:
  float vmax, vmin; //アンチワインドアップ発動の閾値(PWMのduty比の最大・最小値)
  float u, pre_e, err_sum; //uは出力
  float P, I, D, D_pre; //各項の値
  uint8_t mode; //動作モード
};
