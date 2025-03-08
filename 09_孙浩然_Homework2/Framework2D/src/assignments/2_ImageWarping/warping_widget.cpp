#include "warping_widget.h"
#include "warper/IDW_warper.h"
#include "warper/RBF_warper.h"
#include <warper/NN_warper.h>
#include <cmath>
#include <iostream>
#include "annoylib.h"
#include "kissrandom.h"
using namespace Annoy;
namespace USTC_CG
{
using uchar = unsigned char;

WarpingWidget::WarpingWidget(const std::string& label, const std::string& filename)
    : ImageWidget(label, filename)
{
    if (data_)
        back_up_ = std::make_shared<Image>(*data_);
}

void WarpingWidget::draw()
{
    // Draw the image
    ImageWidget::draw();
    // Draw the canvas
    if (flag_enable_selecting_points_)
        select_points();
}
bool is_hole(std::vector<unsigned char> pixel, int threshold = 5)
{
    return pixel[0] < threshold && pixel[1] < threshold && pixel[2] < threshold;
}

std::vector<unsigned char> interpolate(Image& img, int x, int y)
{
    std::vector<unsigned char> result(3, 0);
    int count = 0;

    std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}
    };

    for (auto [dx, dy] : directions)
    {
        int nx = x + dx;
        int ny = y + dy;

        if (nx >= 0 && nx < img.width() && ny >= 0 && ny < img.height())
        {
            std::vector<unsigned char> neighbor = img.get_pixel(nx, ny);
            if (!is_hole(neighbor))
            {
                for (int c = 0; c < 3; ++c)
                {
                    result[c] += neighbor[c];
                }
                count++;
            }
        }
    }

    if (count > 0)
    {
        for (int c = 0; c < 3; ++c)
        {
            result[c] /= count;
        }
    }
    return result;
}



void WarpingWidget::invert()
{
    for (int i = 0; i < data_->width(); ++i)
    {
        for (int j = 0; j < data_->height(); ++j)
        {
            const auto color = data_->get_pixel(i, j);
            data_->set_pixel(
                i,
                j,
                { static_cast<uchar>(255 - color[0]),
                  static_cast<uchar>(255 - color[1]),
                  static_cast<uchar>(255 - color[2]) });
        }
    }
    // After change the image, we should reload the image data to the renderer
    update();
}
void WarpingWidget::mirror(bool is_horizontal, bool is_vertical)
{
    Image image_tmp(*data_);
    int width = data_->width();
    int height = data_->height();

    if (is_horizontal)
    {
        if (is_vertical)
        {
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    data_->set_pixel(
                        i,
                        j,
                        image_tmp.get_pixel(width - 1 - i, height - 1 - j));
                }
            }
        }
        else
        {
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    data_->set_pixel(
                        i, j, image_tmp.get_pixel(width - 1 - i, j));
                }
            }
        }
    }
    else
    {
        if (is_vertical)
        {
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    data_->set_pixel(
                        i, j, image_tmp.get_pixel(i, height - 1 - j));
                }
            }
        }
    }

    // After change the image, we should reload the image data to the renderer
    update();
}
bool is_valid_pixel(const std::vector<unsigned char>& p) {
    return !(p[0] == 255 && p[1] == 0 && p[2] == 255);  // 黑色像素无效
}
void WarpingWidget::gray_scale()
{
    for (int i = 0; i < data_->width(); ++i)
    {
        for (int j = 0; j < data_->height(); ++j)
        {
            const auto color = data_->get_pixel(i, j);
            uchar gray_value = (color[0] + color[1] + color[2]) / 3;
            data_->set_pixel(i, j, { gray_value, gray_value, gray_value });
        }
    }
    // After change the image, we should reload the image data to the renderer
    update();
}
void WarpingWidget::warping()
{
    // HW2_TODO: You should implement your own warping function that interpolate
    // the selected points.
    // Please design a class for such warping operations, utilizing the
    // encapsulation, inheritance, and polymorphism features of C++. 

    // Create a new image to store the result
    Image warped_image(*data_);
    // Initialize the color of result image
    for (int y = 0; y < data_->height(); ++y)
    {
        for (int x = 0; x < data_->width(); ++x)
        {
            warped_image.set_pixel(x, y, { 0, 0, 0 });
        }
    }

    switch (warping_type_)
    {
        case kDefault: break;
        case kFisheye:
        {
            // Example: (simplified) "fish-eye" warping
            // For each (x, y) from the input image, the "fish-eye" warping
            // transfer it to (x', y') in the new image: Note: For this
            // transformation ("fish-eye" warping), one can also calculate the
            // inverse (x', y') -> (x, y) to fill in the "gaps".
            for (int y = 0; y < data_->height(); ++y)
            {
                for (int x = 0; x < data_->width(); ++x)
                {
                    // Apply warping function to (x, y), and we can get (x', y')
                    auto [new_x, new_y] =
                        fisheye_warping(x, y, data_->width(), data_->height());
                    // Copy the color from the original image to the result
                    // image
                    if (new_x >= 0 && new_x < data_->width() && new_y >= 0 &&
                        new_y < data_->height())
                    {
                        std::vector<unsigned char> pixel =
                            data_->get_pixel(x, y);
                        warped_image.set_pixel(new_x, new_y, pixel);
                    }
                }
            }
            break;
        }

        case kIDW:
        {
            Image warped_image_(*data_);
            std::cout << "IDW warping..." << std::endl;
        
            // 1. 创建 IDW 对象
            USTC_CG::IDWWarper idw_warper;
        
            // 2. 设置控制点
            std::vector<std::pair<float, float>> start, end;
            for (size_t i = 0; i < start_points_.size(); ++i)
            {
                start.emplace_back(start_points_[i].x, start_points_[i].y);
                end.emplace_back(end_points_[i].x, end_points_[i].y);
            }
            idw_warper.get_size(data_->width(),data_->height());
            idw_warper.set_control_points(start, end);
            int f = 2;  // 坐标维度用以算距离
            AnnoyIndex<int, float, Euclidean, Kiss32Random, AnnoyIndexSingleThreadedBuildPolicy> index(f);
            std::vector<std::vector<bool>> visited(warped_image.height(), std::vector<bool>(warped_image.width(), false));//相当于创建一个二维数组，用来标记像素是否被访问过，初始化为false
            float max_distance = 20.0f;  // 只考虑一定像素范围内的像素
            
            // 1. 初始化标记像素 (0, 0, 0)
            for (int j = 0; j < warped_image.height(); ++j) {
                for (int i = 0; i < warped_image.width(); ++i) {
                    warped_image.set_pixel(i, j, {0, 0, 0});//set_pixel先列后行
                }
            }            
            // 2. 正向映射
            for (int y = 0; y < data_->height(); ++y) {
                for (int x = 0; x < data_->width(); ++x) {
                    auto [new_x, new_y] = idw_warper.warp(static_cast<float>(x), static_cast<float>(y));
                    int src_x = static_cast<int>(std::round(new_x));
                    
                    int src_y = static_cast<int>(std::round(new_y));
            
                    if (src_x >= 0 && src_x < warped_image.width() && src_y >= 0 && src_y < warped_image.height()) {
                        auto pixel = data_->get_pixel(x, y);
                        warped_image.set_pixel(src_x, src_y, pixel);
                        visited[src_y][src_x] = true;//被映射到的值标记为true
                    }
                }
            }
            
            // 3.建索引
            for (int j = 0; j < warped_image.height(); ++j) {
                for (int i = 0; i < warped_image.width(); ++i) {
                    if (visited[j][i]) {
                        std::vector<float> vec(f);
                        vec[0] = i / float(warped_image.width());
                        vec[1] = j / float(warped_image.height());
                        index.add_item(j * warped_image.width() + i, vec.data());                        
                    }
                }
            }
            index.build(10);
            
            for (int j = 0; j < warped_image.height(); ++j) {
                for (int i = 0; i < warped_image.width(); ++i) {
                    if (!visited[j][i] ) {
                        std::vector<int> closest;
                        std::vector<float> distances;
                        std::vector<float> tmp = {i / float(warped_image.width()), j / float(warped_image.height())};
                        index.get_nns_by_vector(tmp.data(), 10, -1,&closest, &distances);
                        int count = 0;
                        // 直接走权重插值
                        if (!closest.empty()) {
                            float weight_sum = 0.0f;
                            std::vector<float> mean_pixel(3, 0.0f);
                            
                            for (size_t k = 0; k < closest.size(); ++k) {
                                if (distances[k] > max_distance) continue;  
                                float weight = 1.0f / (distances[k] + 1e-6f); // 防止除0
                                weight_sum += weight;         
                                int id = closest[k];
                                int x = static_cast<int>(std::round(id % warped_image.width()));
                                int y = static_cast<int>(std::round(id / warped_image.width())); //必须使用std::round()函数来进行四舍五入，否则就会出现2.999强制转换为2的情况。      
                                auto neighbor = warped_image.get_pixel(x, y);           
                                for (int z = 0; z < 3; ++z) {
                                    mean_pixel[z] += neighbor[z]*weight; 
                                }
                                count++;
                            
                            }
                            warped_image.set_pixel(i, j, {
                                static_cast<unsigned char>(std::round(mean_pixel[0]/weight_sum)),
                                static_cast<unsigned char>(std::round(mean_pixel[1]/weight_sum)),
                                static_cast<unsigned char>(std::round(mean_pixel[2]/weight_sum))
                            });          

                        }
                    }
                }
            }           
            break;
        }
        
        case kRBF:
        {
            Image warped_image_(*data_);
            std::cout << "RBF warping..." << std::endl;
        
            // 1. 创建 RBF对象
            USTC_CG::RBFWarper rbf_warper;
        
            // 2. 设置控制点
            std::vector<std::pair<float, float>> start, end;
            for (size_t i = 0; i < start_points_.size(); ++i)
            {
                start.emplace_back(start_points_[i].x, start_points_[i].y);
                end.emplace_back(end_points_[i].x, end_points_[i].y);
            }
            //rbf_warper.get_size(data_->width(),data_->height());
            rbf_warper.set_control_points(start, end);
            int f = 2;  // 坐标维度用以算距离
            AnnoyIndex<int, float, Euclidean, Kiss32Random, AnnoyIndexSingleThreadedBuildPolicy> index(f);
            std::vector<std::vector<bool>> visited(warped_image.height(), std::vector<bool>(warped_image.width(), false));//相当于创建一个二维数组，用来标记像素是否被访问过，初始化为false
            float max_distance = 20.0f;  // 只考虑一定像素范围内的像素
            
            // 1. 初始化标记像素 (0, 0, 0)
            for (int j = 0; j < warped_image.height(); ++j) {
                for (int i = 0; i < warped_image.width(); ++i) {
                    warped_image.set_pixel(i, j, {0, 0, 0});//set_pixel先列后行
                }
            }            
            // 2. 正向映射
            for (int y = 0; y < data_->height(); ++y) {
                for (int x = 0; x < data_->width(); ++x) {
                    auto [new_x, new_y] = rbf_warper.warp(static_cast<float>(x), static_cast<float>(y));
                    int src_x = static_cast<int>(std::round(new_x));
                    
                    int src_y = static_cast<int>(std::round(new_y));
            
                    if (src_x >= 0 && src_x < warped_image.width() && src_y >= 0 && src_y < warped_image.height()) {
                        auto pixel = data_->get_pixel(x, y);
                        warped_image.set_pixel(src_x, src_y, pixel);
                        visited[src_y][src_x] = true;//被映射到的值标记为true
                    }
                }
            }
            
            // 3.建索引
            for (int j = 0; j < warped_image.height(); ++j) {
                for (int i = 0; i < warped_image.width(); ++i) {
                    if (visited[j][i]) {
                        std::vector<float> vec(f);
                        vec[0] = i / float(warped_image.width());
                        vec[1] = j / float(warped_image.height());
                        index.add_item(j * warped_image.width() + i, vec.data());                        
                    }
                }
            }
            index.build(10);
            
            for (int j = 0; j < warped_image.height(); ++j) {
                for (int i = 0; i < warped_image.width(); ++i) {
                    if (!visited[j][i] ) {
                        std::vector<int> closest;
                        std::vector<float> distances;
                        std::vector<float> tmp = {i / float(warped_image.width()), j / float(warped_image.height())};
                        index.get_nns_by_vector(tmp.data(), 10, -1,&closest, &distances);
                        int count = 0;
                        // 直接走权重插值
                        if (!closest.empty()) {
                            float weight_sum = 0.0f;
                            std::vector<float> mean_pixel(3, 0.0f);
                            
                            for (size_t k = 0; k < closest.size(); ++k) {
                                if (distances[k] > max_distance) continue;  
                                float weight = 1.0f / (distances[k] + 1e-6f); // 防止除0
                                weight_sum += weight;         
                                int id = closest[k];
                                int x = static_cast<int>(std::round(id % warped_image.width()));
                                int y = static_cast<int>(std::round(id / warped_image.width())); //必须使用std::round()函数来进行四舍五入，否则就会出现2.999强制转换为2的情况。      
                                auto neighbor = warped_image.get_pixel(x, y);           
                                for (int z = 0; z < 3; ++z) {
                                    mean_pixel[z] += neighbor[z]*weight; 
                                }
                                count++;
                            
                            }
                            warped_image.set_pixel(i, j, {
                                static_cast<unsigned char>(std::round(mean_pixel[0]/weight_sum)),
                                static_cast<unsigned char>(std::round(mean_pixel[1]/weight_sum)),
                                static_cast<unsigned char>(std::round(mean_pixel[2]/weight_sum))
                            });          

                        }
                    }
                }
            }   
            break;
        }
        case kNN:
        {
            Image warped_image_(*data_);
            std::cout << "NN warping..." << std::endl;
        
            // 1. 创建 IDW 对象
            USTC_CG::NNWarper nn_warper;
        
            // 2. 设置控制点
            std::vector<std::pair<float, float>> start, end;
            for (size_t i = 0; i < start_points_.size(); ++i)
            {
                start.emplace_back(start_points_[i].x, start_points_[i].y);
                end.emplace_back(end_points_[i].x, end_points_[i].y);
            }
            //nn_warper.get_size(data_->width(),data_->height());
            nn_warper.set_control_points(start, end);
            nn_warper.train();
            int f = 2;  // 坐标维度用以算距离
            AnnoyIndex<int, float, Euclidean, Kiss32Random, AnnoyIndexSingleThreadedBuildPolicy> index(f);
            std::vector<std::vector<bool>> visited(warped_image.height(), std::vector<bool>(warped_image.width(), false));//相当于创建一个二维数组，用来标记像素是否被访问过，初始化为false
            float max_distance = 20.0f;  // 只考虑一定像素范围内的像素
            
            // 1. 初始化标记像素 (0, 0, 0)
            for (int j = 0; j < warped_image.height(); ++j) {
                for (int i = 0; i < warped_image.width(); ++i) {
                    warped_image.set_pixel(i, j, {0, 0, 0});//set_pixel先列后行
                }
            }            
            // 2. 正向映射
            for (int y = 0; y < data_->height(); ++y) {
                for (int x = 0; x < data_->width(); ++x) {
                    auto [new_x, new_y] = nn_warper.warp(static_cast<float>(x), static_cast<float>(y));
                    int src_x = static_cast<int>(std::round(new_x));
                    
                    int src_y = static_cast<int>(std::round(new_y));
            
                    if (src_x >= 0 && src_x < warped_image.width() && src_y >= 0 && src_y < warped_image.height()) {
                        auto pixel = data_->get_pixel(x, y);
                        warped_image.set_pixel(src_x, src_y, pixel);
                        visited[src_y][src_x] = true;//被映射到的值标记为true
                    }
                }
            }
            
            // 3.建索引
            for (int j = 0; j < warped_image.height(); ++j) {
                for (int i = 0; i < warped_image.width(); ++i) {
                    if (visited[j][i]) {
                        std::vector<float> vec(f);
                        vec[0] = i / float(warped_image.width());
                        vec[1] = j / float(warped_image.height());
                        index.add_item(j * warped_image.width() + i, vec.data());                        
                    }
                }
            }
            index.build(10);
            
            for (int j = 0; j < warped_image.height(); ++j) {
                for (int i = 0; i < warped_image.width(); ++i) {
                    if (!visited[j][i] ) {
                        std::vector<int> closest;
                        std::vector<float> distances;
                        std::vector<float> tmp = {i / float(warped_image.width()), j / float(warped_image.height())};
                        index.get_nns_by_vector(tmp.data(), 10, -1,&closest, &distances);
                        int count = 0;
                        // 直接走权重插值
                        if (!closest.empty()) {
                            float weight_sum = 0.0f;
                            std::vector<float> mean_pixel(3, 0.0f);
                            
                            for (size_t k = 0; k < closest.size(); ++k) {
                                if (distances[k] > max_distance) continue;  
                                float weight = 1.0f / (distances[k] + 1e-6f); // 防止除0
                                weight_sum += weight;         
                                int id = closest[k];
                                int x = static_cast<int>(std::round(id % warped_image.width()));
                                int y = static_cast<int>(std::round(id / warped_image.width())); //必须使用std::round()函数来进行四舍五入，否则就会出现2.999强制转换为2的情况。      
                                auto neighbor = warped_image.get_pixel(x, y);           
                                for (int z = 0; z < 3; ++z) {
                                    mean_pixel[z] += neighbor[z]*weight; 
                                }
                                count++;
                            
                            }
                            warped_image.set_pixel(i, j, {
                                static_cast<unsigned char>(std::round(mean_pixel[0]/weight_sum)),
                                static_cast<unsigned char>(std::round(mean_pixel[1]/weight_sum)),
                                static_cast<unsigned char>(std::round(mean_pixel[2]/weight_sum))
                            });          

                        }
                    }
                }
            }           
            break;
        }
        default: break;
    }

    *data_ = std::move(warped_image);
    update();
}
void WarpingWidget::restore()
{
    *data_ = *back_up_;
    update();
}
void WarpingWidget::set_default()
{
    warping_type_ = kDefault;
}
void WarpingWidget::set_fisheye()
{
    warping_type_ = kFisheye;
}
void WarpingWidget::set_IDW()
{
    warping_type_ = kIDW;
}
void WarpingWidget::set_RBF()
{
    warping_type_ = kRBF;
}
void WarpingWidget::set_NN()
{
    warping_type_ = kNN;
}   
void WarpingWidget::enable_selecting(bool flag)
{
    flag_enable_selecting_points_ = flag;
}
void WarpingWidget::select_points()
{
    /// Invisible button over the canvas to capture mouse interactions.
    ImGui::SetCursorScreenPos(position_);
    ImGui::InvisibleButton(
        label_.c_str(),
        ImVec2(
            static_cast<float>(image_width_),
            static_cast<float>(image_height_)),
        ImGuiButtonFlags_MouseButtonLeft);
    // Record the current status of the invisible button
    bool is_hovered_ = ImGui::IsItemHovered();
    // Selections
    ImGuiIO& io = ImGui::GetIO();
    if (is_hovered_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        draw_status_ = true;
        start_ = end_ =
            ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
    }
    if (draw_status_)
    {
        end_ = ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            start_points_.push_back(start_);
            end_points_.push_back(end_);
            draw_status_ = false;
        }
    }
    // Visualization
    auto draw_list = ImGui::GetWindowDrawList();
    for (size_t i = 0; i < start_points_.size(); ++i)
    {
        ImVec2 s(
            start_points_[i].x + position_.x, start_points_[i].y + position_.y);
        ImVec2 e(
            end_points_[i].x + position_.x, end_points_[i].y + position_.y);
        draw_list->AddLine(s, e, IM_COL32(255, 0, 0, 255), 2.0f);
        draw_list->AddCircleFilled(s, 4.0f, IM_COL32(0, 0, 255, 255));
        draw_list->AddCircleFilled(e, 4.0f, IM_COL32(0, 255, 0, 255));
    }
    if (draw_status_)
    {
        ImVec2 s(start_.x + position_.x, start_.y + position_.y);
        ImVec2 e(end_.x + position_.x, end_.y + position_.y);
        draw_list->AddLine(s, e, IM_COL32(255, 0, 0, 255), 2.0f);
        draw_list->AddCircleFilled(s, 4.0f, IM_COL32(0, 0, 255, 255));
    }
}
void WarpingWidget::init_selections()
{
    start_points_.clear();
    end_points_.clear();
}

std::pair<int, int>
WarpingWidget::fisheye_warping(int x, int y, int width, int height)
{
    float center_x = width / 2.0f;
    float center_y = height / 2.0f;
    float dx = x - center_x;
    float dy = y - center_y;
    float distance = std::sqrt(dx * dx + dy * dy);

    // Simple non-linear transformation r -> r' = f(r)
    float new_distance = std::sqrt(distance) * 10;

    if (distance == 0)
    {
        return { static_cast<int>(center_x), static_cast<int>(center_y) };
    }
    // (x', y')
    float ratio = new_distance / distance;
    int new_x = static_cast<int>(center_x + dx * ratio);
    int new_y = static_cast<int>(center_y + dy * ratio);

    return { new_x, new_y };
}
}  // namespace USTC_CG