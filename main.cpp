#include <iostream>
#include <math.h>
#include "EigenLidan.h"
#include "MathLidan.h"
#include "geometry.h"
#include "util.h"
#include "RenderWindow.h"
#include "FileReader.h"
#define SDL_MAIN_HANDLED
#include <SDL2/SDL_main.h>

void printUsage()
{
    fprintf(stdout, "Usage: -shader <shader.frag> -channels <channel1.png> <channel2.png> ....");
}

void getSpheres(std::vector<float>& vertices, std::vector<int>& indices, std::vector<Vec3> randPos, std::vector<float> radius)
{
    assert(randPos.size() == radius.size()) ;
    for(int i = 0 ; i < randPos.size() ;i++) {
        drawSphere(vertices, indices, randPos[i], radius[i], 100, 100);
    }
}


int main(int argc, char *argv[])
{
    std::vector<float> vertices ;
    std::vector<int> indices ;
    std::vector<Vec3> randPos ;
    std::vector<float> radius ;

    randPos.push_back(Vec3(0,0,0)) ;
    radius.push_back(1) ;
    getSpheres(vertices, indices, randPos, radius) ;


    SDL_LogSetPriority(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_VERBOSE);

    if (SDL_Init(SDL_INIT_EVERYTHING) == -1)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Initilizated SDL Failed: %s", SDL_GetError());
        return -1;
    }

    bool hasShader = false;
    bool hasTexture = false;

    std::vector<const char *> texFiles;
    std::string shaderSource = "";

    if (argc <= 1)
    {
        printUsage();
        return EXIT_FAILURE;
    }

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "-shader") == 0 && i + 1 < argc)
        {
            std::string t = argv[i+1] ;
            shaderSource = FileReader::readAll(t);
            hasShader = true;
            i++;
        }


        if (strcmp(argv[i], "-channels") == 0)
        {
            int j = 0;
            for (j = i + 1; j < argc; j++)
            {
                std::string t = argv[i] ;
                std::string ext = FileReader::fileExtName(t);

                if (ext.compare( "png") == 0 || ext.compare( "bmp") == 0)
                {
                    hasTexture = true;
                    texFiles.push_back(argv[j]);
                }
                if (texFiles.size() >= 4)
                    break;
            }

            i += j;
        }
    }


    if (hasShader == false)
    {
        printUsage();
        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "No specified fragment shader source file\n");
        return EXIT_FAILURE;
    }

    if (shaderSource == "")
    {
        SDL_LogDebug(SDL_LOG_CATEGORY_APPLICATION, "Read fragment shader file failed\n");
        return EXIT_FAILURE;
    }

    for(int i = 0 ; i< vertices.size()/3;i++)
    {
        std::cout<<"v "<<i<<" "<<vertices[i*3]<<" "<<vertices[i*3+1]<<" "<<vertices[i*3+2]<<std::endl ;
    }

    RenderWindow shaderToy;
    shaderToy.addTexture(texFiles);
    shaderToy.addUserFragmentMainCode(shaderSource.c_str());
    shaderToy.run(vertices.data(),vertices.size()*sizeof(GLfloat),indices.data(),indices.size()*sizeof(int));


    SDL_Quit();
    return (0);
}
