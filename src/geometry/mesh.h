#pragma once
#include "../utils/config.h"

class Mesh {
    public:
        inline void setPositions(std::vector<float> positions) { this-> positions = positions;}
        inline void setColors(std::vector<float> colors) { this-> colors = colors;}
        inline void setIndices(std::vector<unsigned int> indices) { this-> indices = indices;}

        inline std::vector<unsigned int> getIndices() {return indices;}

        void initGpuGeometry();
        void draw();
        ~Mesh();

    private:
    
        std::vector<float> positions;
        std::vector<float> colors;
        std::vector<unsigned int> indices;
        
        GLuint VAO, EBO;
        std::vector<GLuint> VBOs;

};
