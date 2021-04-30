//
// Created by lidan on 2021/3/2.
//

#ifndef VR_RENDERWINDOW_H
#define VR_RENDERWINDOW_H

#include "Window.h"
#include "Timer.h"
#include "ShaderInput.h"
#include "BufferObject.h"
#include "ShaderProgram.h"
#include "VertexArrayObject.h"
#include "util.h"

class RenderWindow :public Window{
private:
    void initilizeUniformValue();
    void bindUniform();

    Timer  timer;
    FShaderInput mInput;
    VertexArrayObject *mVAO;
    BufferObject *mVBOArray;
    BufferObject *mVBOIndex;
    ShaderProgram *mProgram;
    std::vector<Sample2D *> mTextures;
    char title[20];

public:
    RenderWindow();
    ~RenderWindow();

    void addTexture(std::vector<const char *> &texFileNames);
    void addUserFragmentMainCode(const char *fragmentMain);

protected:
    void initilizeGL(float* vertices, int vsize, int* indices, int isize) override;
    void resizeGL(int w, int h) override;
    void renderGL() override;

    void keydownEvent(SDL_KeyboardEvent *event) override;
    void mouseButtonUpEvent(SDL_MouseButtonEvent *event) override;
    void mouseButtonDownEvent(SDL_MouseButtonEvent *event) override;
    void mouseMotionEvent(SDL_MouseMotionEvent *event) override;


};


#endif //VR_RENDERWINDOW_H
