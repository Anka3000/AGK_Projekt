#include <GL/freeglut.h>
#include <cmath>
#include <iostream>
#include <vector>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const int numU = 10, numV = 10;
struct Point {
    GLfloat x, y, z;
    GLfloat vx, vy, vz;
    GLfloat fx, fy, fz;
    bool fixed;
};

std::vector<std::vector<Point>> controlPoints(numU, std::vector<Point>(numV));
GLfloat wind = 0.0f;
GLfloat windDirection = 1.0f;
const GLfloat windStrength = 0.001f;
const GLfloat stiffness = 0.5f;
const GLfloat damping = 0.01f;
GLuint texture;
GLfloat angleX = 0.0f, angleY = 0.0f;
int lastMouseX, lastMouseY;
bool isLeftButtonPressed = false;

void checkGLError(const std::string& msg) {
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cerr << msg << " - OpenGL error: " << gluErrorString(err) << std::endl;
    }
}

void loadTexture() {
    int width, height, nrChannels;
    unsigned char* data = stbi_load("cloth.png", &width, &height, &nrChannels, 0);
    if (data) {
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        if (nrChannels == 3) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        }
        else if (nrChannels == 4) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        }
        else {
            std::cerr << "Unknown number of channels: " << nrChannels << std::endl;
            stbi_image_free(data);
            return;
        }
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        stbi_image_free(data);
    }
    else {
        std::cerr << "Failed to load texture" << std::endl;
    }
    checkGLError("loadTexture");
}

void initControlPoints() 
{
    for (int i = 0; i < numU; ++i) {
        for (int j = 0; j < numV; ++j) {
            GLfloat z = 0.1f * sinf((GLfloat)i / (numU - 1) * 2.0f * M_PI) * cosf((GLfloat)j / (numV - 1) * 2.0f * M_PI);
            controlPoints[i][j] = { (GLfloat)(i - numU / 2), (GLfloat)(j - numV / 2), z, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false };
        }
    }

    controlPoints[0][0].fixed = true;
    controlPoints[0][numV - 1].fixed = true;
}

void applyForce(Point& p, GLfloat fx, GLfloat fy, GLfloat fz) {
    p.fx += fx;
    p.fy += fy;
    p.fz += fz;
}

void updateControlPoints() {
    for (int i = 0; i < numU; ++i) {
        for (int j = 0; j < numV; ++j) {
            controlPoints[i][j].fx = 0.0f;
            controlPoints[i][j].fy = 0.0f;
            controlPoints[i][j].fz = 0.0f;
        }
    }

    for (int i = 0; i < numU; ++i) {
        for (int j = 0; j < numV; ++j) {
            applyForce(controlPoints[i][j], windStrength * windDirection, 0.0f, 0.0f);
        }
    }

    for (int i = 0; i < numU; ++i) {
        for (int j = 0; j < numV; ++j) {
            if (i < numU - 1) {
                GLfloat dx = controlPoints[i + 1][j].x - controlPoints[i][j].x;
                GLfloat dy = controlPoints[i + 1][j].y - controlPoints[i][j].y;
                GLfloat dz = controlPoints[i + 1][j].z - controlPoints[i][j].z;
                GLfloat dist = sqrtf(dx * dx + dy * dy + dz * dz);
                GLfloat force = stiffness * (dist - 1.0f);
                applyForce(controlPoints[i][j], force * dx / dist, force * dy / dist, force * dz / dist);
                applyForce(controlPoints[i + 1][j], -force * dx / dist, -force * dy / dist, -force * dz / dist);
            }
            if (j < numV - 1) {
                GLfloat dx = controlPoints[i][j + 1].x - controlPoints[i][j].x;
                GLfloat dy = controlPoints[i][j + 1].y - controlPoints[i][j].y;
                GLfloat dz = controlPoints[i][j + 1].z - controlPoints[i][j].z;
                GLfloat dist = sqrtf(dx * dx + dy * dy + dz * dz);
                GLfloat force = stiffness * (dist - 1.0f);
                applyForce(controlPoints[i][j], force * dx / dist, force * dy / dist, force * dz / dist);
                applyForce(controlPoints[i][j + 1], -force * dx / dist, -force * dy / dist, -force * dz / dist);
            }
        }
    }

    for (int i = 0; i < numU; ++i) {
        for (int j = 0; j < numV; ++j) {
            if (!controlPoints[i][j].fixed) {
                controlPoints[i][j].vx += controlPoints[i][j].fx;
                controlPoints[i][j].vy += controlPoints[i][j].fy;
                controlPoints[i][j].vz += controlPoints[i][j].fz;
                controlPoints[i][j].vx *= (1.0f - damping);
                controlPoints[i][j].vy *= (1.0f - damping);
                controlPoints[i][j].vz *= (1.0f - damping);
                controlPoints[i][j].x += controlPoints[i][j].vx;
                controlPoints[i][j].y += controlPoints[i][j].vy;
                controlPoints[i][j].z += controlPoints[i][j].vz;
            }
        }
    }
}

Point bezier(const Point& p0, const Point& p1, const Point& p2, float t) {
    Point result;
    result.x = (1 - t) * (1 - t) * p0.x + 2 * (1 - t) * t * p1.x + t * t * p2.x;
    result.y = (1 - t) * (1 - t) * p0.y + 2 * (1 - t) * t * p1.y + t * t * p2.y;
    result.z = (1 - t) * (1 - t) * p0.z + 2 * (1 - t) * t * p1.z + t * t * p2.z;
    return result;
}

Point bezierSurface(const std::vector<std::vector<Point>>& controlPoints, float u, float v) {
    std::vector<Point> tempU(numU);
    for (int i = 0; i < numU; ++i) {
        tempU[i] = bezier(controlPoints[i][0], controlPoints[i][numV / 2], controlPoints[i][numV - 1], v);
    }
    return bezier(tempU[0], tempU[numU / 2], tempU[numU - 1], u);
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 25.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    glRotatef(angleX, 1.0f, 0.0f, 0.0f);
    glRotatef(angleY, 0.0f, 1.0f, 0.0f);

    updateControlPoints();

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);

    glPushMatrix();
    glTranslatef(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < numU - 1; ++i) {
        for (int j = 0; j < numV - 1; ++j) {
            float u = (float)i / (numU - 1);
            float v = (float)j / (numV - 1);
            float uNext = (float)(i + 1) / (numU - 1);
            float vNext = (float)(j + 1) / (numV - 1);

            Point p0 = bezierSurface(controlPoints, u, v);
            Point p1 = bezierSurface(controlPoints, uNext, v);
            Point p2 = bezierSurface(controlPoints, u, vNext);
            Point p3 = bezierSurface(controlPoints, uNext, vNext);

            glBegin(GL_TRIANGLES);

            glTexCoord2f(u, v);
            glVertex3f(p0.x, p0.y, p0.z);

            glTexCoord2f(uNext, v);
            glVertex3f(p1.x, p1.y, p1.z);

            glTexCoord2f(u, vNext);
            glVertex3f(p2.x, p2.y, p2.z);

            glTexCoord2f(uNext, v);
            glVertex3f(p1.x, p1.y, p1.z);

            glTexCoord2f(uNext, vNext);
            glVertex3f(p3.x, p3.y, p3.z);

            glTexCoord2f(u, vNext);
            glVertex3f(p2.x, p2.y, p2.z);

            glEnd();
        }
    }

    glPopMatrix();
    glDisable(GL_TEXTURE_2D);
    glutSwapBuffers();
    checkGLError("display");
}

void reshape(int w, int h) {
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (GLfloat)w / (GLfloat)h, 1.0, 200.0);
    glMatrixMode(GL_MODELVIEW);
    checkGLError("reshape");
}

void timer(int) {
    wind += 0.01f * windDirection;
    if (wind > 2.0f || wind < -2.0f) {
        windDirection = -windDirection;
    }
    glutPostRedisplay();
    glutTimerFunc(16, timer, 0);
    checkGLError("timer");
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            isLeftButtonPressed = true;
            lastMouseX = x;
            lastMouseY = y;
        }
        else if (state == GLUT_UP) {
            isLeftButtonPressed = false;
        }
    }
}

void motion(int x, int y) {
    if (isLeftButtonPressed) {
        angleY += (x - lastMouseX);
        angleX += (y - lastMouseY);
        lastMouseX = x;
        lastMouseY = y;
        glutPostRedisplay();
    }
}

void init() {
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.15, 0.14, 0.3, 1.0);

    initControlPoints();
    loadTexture();

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat light_position0[] = { 1.0, 1.0, 1.0, 0.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
    GLfloat light_diffuse0[] = { 1.0, 1.0, 1.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse0);
    GLfloat light_specular0[] = { 1.0, 1.0, 1.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular0);
    GLfloat light_ambient0[] = { 0.2, 0.2, 0.2, 1.0 };
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient0);

    glEnable(GL_LIGHT1);
    GLfloat light_position1[] = { 1.0, -1.0, 1.0, 0.0 };
    glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
    GLfloat light_diffuse1[] = { 0.5, 0.5, 0.5, 1.0 };
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse1);
    GLfloat light_specular1[] = { 0.5, 0.5, 0.5, 1.0 };
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular1);
    GLfloat light_ambient1[] = { 0.1, 0.1, 0.1, 1.0 };
    glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient1);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialf(GL_FRONT, GL_SHININESS, 50.0);
    checkGLError("init");
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Cloth Simulation");

    init();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutTimerFunc(0, timer, 0);
    glutMainLoop();
    return 0;
}
