#define NORMAL 1
#define BUMPLESS 2
#define NORMAL_WITH_FF 3
#define BUMPLESS_WITH_FF 4

typedef struct PIDSettings{
  float kp, ki, kd, ts, tdel, vmax, vmin, T, K;
  uint8_t mode;
};

class PIDCalculator{
public:
  PIDCalculator(PIDSettings);
  float calcValue(float); //誤差から出力を計算
  
private:
  float kp, ki, kd, ts, tdel, T, K; //T, Kはフィードフォワード制御用の定数
  float vmax, vmin; //アンチワインドアップ発動の閾値(PWMのduty比の最大・最小値)
  float u, pre_e, err_sum; //uは出力
  float P, I, D, D_pre; //各項の値
  uint8_t mode; //動作モード
};
