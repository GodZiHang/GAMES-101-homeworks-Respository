#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle1, float rotation_angle2, float rotation_angle3)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    rotation_angle1 = rotation_angle1 / 180 * M_PI;
    rotation_angle2 = rotation_angle2 / 180 * M_PI;
    rotation_angle3 = rotation_angle3 / 180 * M_PI;
    Eigen::Matrix4f translate1;
    Eigen::Matrix4f translate2;
    Eigen::Matrix4f translate3;
    translate1 << cos(rotation_angle1), -sin(rotation_angle1), 0, 0, \
                sin(rotation_angle1), cos(rotation_angle1), 0, 0, \
                0, 0, 1, 0, \
                0, 0, 0, 1;
    translate2 << 1, 0, 0, 0, \
                0,cos(rotation_angle2), -sin(rotation_angle2), 0, \
                0, sin(rotation_angle2), cos(rotation_angle2), 0, \
                0, 0, 0, 1;
    translate3 << cos(rotation_angle3), 0, sin(rotation_angle3), 0, \
                0, 1, 0, 0, \
                -sin(rotation_angle3), 0, cos(rotation_angle3), 0, \
                0, 0, 0, 1;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    model = translate1 * model;
    model = translate2 * model;
    model = translate3 * model;
    return model;
}

Eigen::Matrix4f get_model_matrix2(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    rotation_angle = rotation_angle / 180 * M_PI;
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0, \
                0,cos(rotation_angle), -sin(rotation_angle), 0, \
                0, sin(rotation_angle), cos(rotation_angle), 0, \
                0, 0, 0, 1;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    model = translate * model;

    return model;
}

Eigen::Matrix4f get_model_matrix3(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    rotation_angle = rotation_angle / 180 * M_PI;
    Eigen::Matrix4f translate;
    translate << cos(rotation_angle), 0, sin(rotation_angle), 0, \
                0, 1, 0, 0, \
                -sin(rotation_angle), 0, cos(rotation_angle), 0, \
                0, 0, 0, 1;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    model = translate * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    eye_fov = eye_fov / 180.0f * MY_PI;
    float h = 2.0f*tan(eye_fov/2.0f)*zNear;
    float w = h/aspect_ratio;
    Eigen::Matrix4f translate;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    //Eigen::Matrix4f M1; 
    //M1 << zNear, 0, 0, 0, \
    //    0, zNear, 0, 0, \
    //    0, 0, zNear + zFar, zNear*zFar, \
    //    0, 0, 1, 0;
    //Eigen::Matrix4f M2 = Eigen::Matrix4f::Identity();
    //M2(0, 0) = 2.0f/w;
    //M2(1, 1) = 2.0f/h;
    //M2(2, 2) = 2.0f/(zFar-zNear);
    //Eigen::Matrix4f M3 = Eigen::Matrix4f::Identity();
    //M3(2, 3) = (zNear-zFar)/2.0f;
    //translate = M3*M2*M1;
    translate << 2.f*zNear/w, 0.f, 0.f, 0.f, \
                0.f, 2.f*zNear/h, 0.f, 0.f, \
               0.f, 0.f, ((-zFar+zNear)+4.f*(zFar+zNear))/2.f*(zFar-zNear), 2.f*zNear*zFar/(zNear-zFar), \
               0.f, 0.f, 1.f, 0.f;
    return translate*projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    angle = angle /180.f * MY_PI;
    float square = axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2];
    axis[0] /= sqrt(square);
    axis[1] /= sqrt(square);
    axis[2] /= sqrt(square);
    Eigen::Matrix3f k;
    k << 0.f, -axis[2], axis[1], \
        axis[2], 0.f, -axis[0], \
        -axis[1], axis[0], 0.f;
    Eigen::Matrix3f E = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f rotationMat;
    rotationMat = E*cos(angle) + sin(angle)*k + (1-cos(angle))*axis*axis.transpose();

    model.block<3, 3>(0, 0) = rotationMat;
    return model;
}

int main(int argc, const char** argv)
{
    float angle = 0, angle2 = 0, angle3 = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }
    //光栅化器构造函数
    rst::rasterizer r(700, 700);
    //视角坐标
    Eigen::Vector3f eye_pos = {0, 0, 5};
    //硬编码的三角形
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, angle2, angle3));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, angle2, angle3));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        switch(key) {
            case 'a':
                angle += 10;
                break;
            case 'd':
                angle -= 10;
                break;
            case 'w':
                angle2 += 10;
                break;
            case 's':
                angle2 -= 10;
                break;
            case 'q':
                angle3 += 10;
                break;
            case 'e':
                angle3 -= 10;
        }
        // if (key == 'a') {
        //     angle += 10;
        // }
        // else if (key == 'd') {
        //     angle -= 10;
        // }
    }

    return 0;
}
