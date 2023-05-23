#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <curl/curl.h>
#include <string>


class MiNodo : public rclcpp::Node
{
public:
    MiNodo()
    : Node("mi_nodo")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MiNodo::listener_callback, this, std::placeholders::_1));
    }

private:
    void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float value = 0.41;
        std::string ip = "192.168.11.198:33333";
        //RCLCPP_INFO(this->get_logger(), "He recibido: [%f, %f]", msg->linear.x, msg->angular.z);
        if (msg->angular.z > value) {
            RCLCPP_INFO(this->get_logger(), "El robot está girando a la izquierda");
            makeGetRequest("http://" + ip + "/right");
        } else if (msg->angular.z < (-1*value)) {
            RCLCPP_INFO(this->get_logger(), "El robot está girando a la derecha");
            makeGetRequest("http://"+ip+"//left");
        } else {
            RCLCPP_INFO(this->get_logger(), "El robot está avanzando recto");
            makeGetRequest("http://"+ip+"//normal");
        }
    }


    void makeGetRequest(const std::string &url) {
        CURL *curl;
        CURLcode res;

        curl_global_init(CURL_GLOBAL_DEFAULT);

        curl = curl_easy_init();
        if(curl) {
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

            // Si quieres que curl muestre su progreso, descomenta la siguiente línea
            // curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);

            res = curl_easy_perform(curl);

            // Comprueba si hubo errores
            if(res != CURLE_OK)
                fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));

            // Limpia todo
            curl_easy_cleanup(curl);
        }

        curl_global_cleanup();
    }


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiNodo>());
    rclcpp::shutdown();
    return 0;
}
