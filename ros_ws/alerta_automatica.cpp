#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Bool.h>
#include <vector>

class AlertaAutomatica {
public:
  explicit AlertaAutomatica(ros::NodeHandle& nh) : nh_(nh) {
    sub_dist_ = nh_.subscribe("/sensor_distances", 1, &AlertaAutomatica::distCallback, this);
    pub_alerta_ = nh_.advertise<std_msgs::Bool>("/alerta_led", 1, false);

    sub_modo_manual_    = nh_.subscribe("/modo_manual", 1, &AlertaAutomatica::modoManualCallback, this);
    sub_alerta_forzada_ = nh_.subscribe("/alerta_forzada", 1, &AlertaAutomatica::alertaForzadaCallback, this);

    ROS_INFO_STREAM("alerta_automatica iniciado | rango=["
                    << kMinRange << "," << kMaxRange
                    << "] | zero_is_empty=" << (kZeroIsEmpty ? "true" : "false")
                    << " | max_valid_mm=" << kMaxValidMm
                    << " | discard_ratio=" << kInvalidRatio
                    << " | trips_set=" << kTripsToSet
                    << " | trips_clear=" << kTripsToClear);
  }

private:
  // --- Constantes (ajusta aquí y recompila) ---
  static constexpr int    kMinRange      = 200;    // mm
  static constexpr int    kMaxRange      = 2000;   // mm
  static constexpr bool   kZeroIsEmpty   = true;   // 0 cuenta como vacío
  static constexpr int    kMaxValidMm    = 6000;   // por encima ⇒ valor imposible/pico
  static constexpr double kInvalidRatio  = 0.50;   // descartar trama si >50% son imposibles
  static constexpr int    kTripsToSet    = 2;      // malas consecutivas para pasar a ROJO
  static constexpr int    kTripsToClear  = 3;      // buenas consecutivas para volver a VERDE

  // --- Utilidades ---
  void maybePublish(bool new_alert) {
    if (!have_last_ || new_alert != last_alerta_) {
      std_msgs::Bool out; out.data = new_alert;
      pub_alerta_.publish(out);
      last_alerta_ = new_alert;
      have_last_   = true;
      ROS_DEBUG("Publico alerta=%s (cambio)", new_alert ? "true" : "false");
    }
  }

  // Devuelve true si TODA la trama es válida y dentro de rango (según reglas).
  // 'discard' se pone a true si la trama parece corrupta (demasiados imposibles).
  bool frameIsOK(const std::vector<uint16_t>& v, bool& discard) {
    discard = false;
    if (v.empty()) return false;

    int invalid_hard = 0;     // > kMaxValidMm
    bool any_bad = false;     // 0 (si aplica) o fuera de [min,max]

    for (auto d : v) {
      if (d == 65535) d = 0;  // por si Arduino no lo normalizó

      if (d > (uint16_t)kMaxValidMm) invalid_hard++;

      if ((kZeroIsEmpty && d == 0) || d < kMinRange || d > kMaxRange)
        any_bad = true;
    }

    const double ratio = static_cast<double>(invalid_hard) / static_cast<double>(v.size());
    if (ratio > kInvalidRatio) {
      discard = true; // ignoramos por completo esta trama
      ROS_DEBUG("Trama descartada: %d/%zu imposibles (%.2f)",
                invalid_hard, v.size(), ratio);
      return last_alerta_; // valor irrelevante si discard=true
    }
    return !any_bad;
  }

  // --- Callbacks ---
  void distCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg) {
    if (modo_manual_) {               // en manual ignoramos distancias
      maybePublish(alerta_forzada_);
      return;
    }

    std::vector<uint16_t> v(msg->data.begin(), msg->data.end());
    bool discard = false;
    const bool ok = frameIsOK(v, discard);
    if (discard) return;              // no cuenta ni como buena ni como mala

    if (ok) {
      bad_streak_ = 0;
      if (good_streak_ < kTripsToClear) good_streak_++;
    } else {
      good_streak_ = 0;
      if (bad_streak_ < kTripsToSet) bad_streak_++;
    }

    bool want_alert = last_alerta_;
    if (!last_alerta_ && bad_streak_ >= kTripsToSet) {
      want_alert = true;              // pasar a ROJO
    } else if (last_alerta_ && good_streak_ >= kTripsToClear) {
      want_alert = false;             // volver a VERDE
    }
    maybePublish(want_alert);
  }

  void modoManualCallback(const std_msgs::Bool::ConstPtr& msg) {
    modo_manual_ = msg->data;
    ROS_INFO("Modo manual: %s", modo_manual_ ? "ON" : "OFF");
    // Si quieres forzar publicación al cambiar de modo:
    // have_last_ = false;
  }

  void alertaForzadaCallback(const std_msgs::Bool::ConstPtr& msg) {
    alerta_forzada_ = msg->data;
    if (modo_manual_) maybePublish(alerta_forzada_);
  }

  // --- ROS ---
  ros::NodeHandle nh_;
  ros::Subscriber sub_dist_, sub_modo_manual_, sub_alerta_forzada_;
  ros::Publisher  pub_alerta_;

  // --- Estado ---
  bool last_alerta_{false};
  bool have_last_{false};
  int  good_streak_{0};
  int  bad_streak_{0};
  bool modo_manual_{false};
  bool alerta_forzada_{false};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "alerta_automatica");
  ros::NodeHandle nh;
  AlertaAutomatica n(nh);
  ros::spin();
  return 0;
}
