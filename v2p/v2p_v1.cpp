#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm> 
#include <cctype>
#include <locale>

#include <Eigen/Dense>

// =====================
// for string opteration
// =====================
// ref: https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
// #include <algorithm> 
// #include <cctype>
// #include <locale>

// trim from start (in place)
void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}

// trim from start (copying)
std::string ltrim_copy(std::string s) {
    ltrim(s);
    return s;
}

// trim from end (copying)
std::string rtrim_copy(std::string s) {
    rtrim(s);
    return s;
}

// trim from both ends (copying)
std::string trim_copy(std::string s) {
    trim(s);
    return s;
}

// ============================
// END OF for string opteration
// ============================

void line_split(const std::string &inputstring, std::vector<std::string> &output, char delim)
{
    size_t pos = inputstring.find(delim);
    size_t initialPos = 0;
    output.clear();

    // Decompose statement
    while (pos != std::string::npos) {
        output.push_back(inputstring.substr(initialPos, pos - initialPos));
        initialPos = pos + 1;

        pos = inputstring.find(delim, initialPos);
    }

    // Add the last one
    output.push_back(inputstring.substr(initialPos, std::min(pos, inputstring.size()) - initialPos + 1));

    return;
}

int main()
{
    //std::string inputfilename = "vertices.txt";
    std::string inputfilename;
    std::vector<std::string> values;
    std::vector<std::vector<float>> vertex_xyz;
    std::string line = "";
    std::ifstream fin;
    std::ofstream fout;
    int vertexNum = 4;
    int wallIdx = 0;
    char delim = '\t';

    // input stage
    //std::cout << "input file name: " << std::endl;
    //std::cin >> inputfilename;
    //inputfilename = "wallvertex_0_0.txt";
    //std::cout << "name the wall index: " << std::endl;
    //std::cin >> wallIdx;

    fout.open("temp_dump.txt", std::ios::out);

    for (int wallIdx = 0; wallIdx < 5; wallIdx++) {
        inputfilename = "wallvertex_0_";
        inputfilename.append(std::to_string(wallIdx)).append(".txt");
        std::cout << inputfilename << std::endl;
        vertex_xyz.resize(4);
        std::cout << "reading 4 vertices" << std::endl;

        fin.open(inputfilename, std::ios::in);
        for (int i = 0; i < vertexNum; i++) {
            getline(fin, line);
            vertex_xyz[i].resize(3);
            line_split(line, values, delim);
            for (int j = 0; j < 3; j++) {
                vertex_xyz[i][j] = std::stof(trim_copy(values[j])) / 100;
                std::cout << vertex_xyz[i][j] << " ";
            }
            std::cout << std::endl;
        }
        fin.close();

        // parameter generation stage
        // ---------------------------
        // width and height
        float _x_diff, _y_diff, _z_diff;
        float _3js_wall_width, _3js_wall_height = 0.0f;
        _x_diff = vertex_xyz[0][0] - vertex_xyz[1][0];
        _y_diff = vertex_xyz[0][1] - vertex_xyz[1][1];
        _z_diff = vertex_xyz[2][2] - vertex_xyz[0][2];

        _3js_wall_width = std::sqrt(_x_diff * _x_diff + _y_diff * _y_diff);
        if (_z_diff > 10)
        {
            _3js_wall_height = std::abs(_z_diff);
        }
        else // the case where the wall is actualy ceiling or ground
        {
            _x_diff = vertex_xyz[1][0] - vertex_xyz[2][0];
            _y_diff = vertex_xyz[1][1] - vertex_xyz[2][1];
            _3js_wall_height = std::sqrt(_x_diff * _x_diff + _y_diff * _y_diff);
        }

        std::cout << std::endl << "width and height: " << _3js_wall_width << ", " << _3js_wall_height << std::endl;

        // ---------------------------
        // x y z offset
        Eigen::MatrixXf A(4, 3);
        Eigen::VectorXf b(4);
        A << vertex_xyz[0][0], vertex_xyz[0][1], vertex_xyz[0][2],
            vertex_xyz[1][0], vertex_xyz[1][1], vertex_xyz[1][2],
            vertex_xyz[2][0], vertex_xyz[2][1], vertex_xyz[2][2],
            vertex_xyz[3][0], vertex_xyz[3][1], vertex_xyz[3][2];
        b << -1, -1, -1, -1;
        Eigen::Vector3f v = A.colPivHouseholderQr().solve(b); // the v here solves for a, b, and c, while d is hardcoded as 1
                    // std::cout << "The solution is:\n" << v << std::endl;
                    // 2.b) distance of wall plane from origin
        float _3js_wall_distance_from_origin = 1.0f / v.norm();
        std::cout << std::endl << "distance of plane from origin: " << _3js_wall_distance_from_origin << std::endl;
        float u[3] = { v[0] / v.norm() , v[1] / v.norm() , v[2] / v.norm() };

        std::vector<float> check_pt_xyz;
        check_pt_xyz.resize(3);
        check_pt_xyz[0] = _3js_wall_distance_from_origin * u[0];
        check_pt_xyz[1] = _3js_wall_distance_from_origin * u[1];
        check_pt_xyz[2] = _3js_wall_distance_from_origin * u[2];

        float normal_vector_direction_checkValue = v[0] * check_pt_xyz[0] + v[1] * check_pt_xyz[1] + v[2] * check_pt_xyz[2] + 1;
        if (normal_vector_direction_checkValue * normal_vector_direction_checkValue < 0.001) {
            std::cout << std::endl << "normal does not need to toggle direction. it is pointing outward from origin" << std::endl;
        }
        else {
            std::cout << std::endl << "normal needs to toggle direction in order to be pointing outward from origin" << std::endl;
            v[0] = -v[0];
            v[1] = -v[1];
            v[2] = -v[2];
            u[0] = v[0] / v.norm();
            u[1] = v[1] / v.norm();
            u[2] = v[2] / v.norm();
        }
        std::cout << "normal vector is " << std::endl << v << std::endl;
        //float u[3] = { v[0] / v.norm() , v[1] / v.norm() , v[2] / v.norm() };
        std::cout << "normalized normal vector is " << std::endl << u[0] << ", " << u[1] << ", " << u[2] << std::endl;

        float _js_offsetX, _js_offsetY, _js_offsetZ;
		float _tempValue_x = 0.0f, _tempValue_y = 0.0f, _tempValue_z = 0.0f;
		for (int i = 0; i < 4; i++) {
			_tempValue_x = _tempValue_x + vertex_xyz[i][0];
			_tempValue_y = _tempValue_y + vertex_xyz[i][1];
			_tempValue_z = _tempValue_z + vertex_xyz[i][2];
		}
		_js_offsetX = _tempValue_x / 4; //_3js_wall_distance_from_origin * u[0];
		_js_offsetY = _tempValue_z / 4; //_3js_wall_distance_from_origin * u[2];
		_js_offsetZ = - _tempValue_y / 4; //_3js_wall_distance_from_origin * - u[1];

        std::cout << std::endl << "offset in 3js coord system:" << std::endl
            << _js_offsetX << ", " << _js_offsetY << ", " << _js_offsetZ << std::endl;

        // ---------------------------
        // rotation angle
        float _js_rotateX, _js_rotateY, _js_rotateZ = 0.0f;
        _js_rotateX = std::atanf(u[2] / u[1]);
        if (_z_diff > 10) {
            _js_rotateY = std::atanf(u[1] / u[0]) - 3.141592654f / 2.0f;
            _js_rotateZ = std::atanf(u[2] / u[0]);
        }
        else {
            _js_rotateY = 0;
            _js_rotateZ = std::atanf(_y_diff / _x_diff) - 3.141592654f / 2.0f;
        }

        // ---------------------------
        // dump 3js code
        fout << "        "
            << "var planeGeometry_0" << wallIdx << " = new THREE.PlaneGeometry(" << _3js_wall_width << "," << _3js_wall_height << ");\n\
        var planeMaterial_0" << wallIdx << " = new THREE.MeshStandardMaterial({ color:0xff0000 });\n\
        var plane_0" << wallIdx << " = new THREE.Mesh(planeGeometry_0" << wallIdx << ", planeMaterial_0" << wallIdx << ");\n\
        // we should perform translation first,\n\
        // w.r.t. original three.js's coordinate system\n\
        // after that we perform rotation of plane in the\n\
        // translated coordinate system\n\
        plane_0" << wallIdx << ".translateX(" << _js_offsetX << ");\n\
        plane_0" << wallIdx << ".translateY(" << _js_offsetY << ");\n\
        plane_0" << wallIdx << ".translateZ(" << _js_offsetZ << ");\n\
        plane_0" << wallIdx << ".rotateX(" << _js_rotateX << ");\n\
        plane_0" << wallIdx << ".rotateY(" << _js_rotateY << ");\n\
        plane_0" << wallIdx << ".rotateZ(" << _js_rotateZ << ");\n\
        plane_0" << wallIdx << ".receiveShadow = true;\n\
        scene.add(plane_0" << wallIdx << ");\n";

        fout << "\n";
    }
    fout.close();
    std::getchar();

    return 0;
}
