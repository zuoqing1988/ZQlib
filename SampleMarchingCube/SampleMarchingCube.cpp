//
// Marching Cubes Example Program 
// by Cory Bloyd (corysama@yahoo.com)
//
// A simple, portable and complete implementation of the Marching Cubes
// and Marching Tetrahedrons algorithms in a single source file.
// There are many ways that this code could be made faster, but the 
// intent is for the code to be easy to understand.
//
// For a description of the algorithm go to
// http://astronomy.swin.edu.au/pbourke/modelling/polygonise/
//
// This code is public domain.
//

#include "windows.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
//This program requires the OpenGL and GLUT libraries
// You can obtain them for free from http://www.opengl.org

#include "ZQ_MarchingCube.h"
#include "GL/glut.h"

using namespace ZQ;


static const float afAmbientWhite [] = {0.25, 0.25, 0.25, 1.00}; 
static const float afAmbientRed   [] = {0.25, 0.00, 0.00, 1.00}; 
static const float afAmbientGreen [] = {0.00, 0.25, 0.00, 1.00}; 
static const float afAmbientBlue  [] = {0.00, 0.00, 0.25, 1.00}; 
static const float afDiffuseWhite [] = {0.75, 0.75, 0.75, 1.00}; 
static const float afDiffuseRed   [] = {0.75, 0.00, 0.00, 1.00}; 
static const float afDiffuseGreen [] = {0.00, 0.75, 0.00, 1.00}; 
static const float afDiffuseBlue  [] = {0.00, 0.00, 0.75, 1.00}; 
static const float afSpecularWhite[] = {1.00, 1.00, 1.00, 1.00}; 
static const float afSpecularRed  [] = {1.00, 0.25, 0.25, 1.00}; 
static const float afSpecularGreen[] = {0.25, 1.00, 0.25, 1.00}; 
static const float afSpecularBlue [] = {0.25, 0.25, 1.00, 1.00}; 

GLenum    ePolygonMode = GL_FILL;
int     iDataSetSize = 128;
float   fStepSize = 1.0/iDataSetSize;
float   fTargetValue = iDataSetSize / 2 + 2;
float   fTime = 0.0;
ZQ_Vec3D  sSourcePoint[3];
GLboolean bSpin = true;
GLboolean bMove = false;
GLboolean bLight = true;






void vIdle();
void vDrawScene(); 
void vResize(GLsizei, GLsizei);
void vKeyboard(unsigned char cKey, int iX, int iY);
void vSpecial(int iKey, int iX, int iY);

void vPrintHelp();
void vSetTime(float fTime);


ZQ_MarchingCube* m_MC = 0;


void main(int argc, char **argv) 
{ 
        float afPropertiesAmbient [] = {0.50, 0.50, 0.50, 1.00}; 
        float afPropertiesDiffuse [] = {0.75, 0.75, 0.75, 1.00}; 
        float afPropertiesSpecular[] = {1.00, 1.00, 1.00, 1.00}; 

        GLsizei iWidth = 640.0; 
        GLsizei iHeight = 480.0; 

		m_MC = new ZQ_MarchingCube();
		m_MC->SetSize(iDataSetSize+1,iDataSetSize+1,iDataSetSize+1);
		float* data = new float[(iDataSetSize+1)*(iDataSetSize+1)*(iDataSetSize+1)];
		for(int i = 0;i < (iDataSetSize+1);i++)
		{
			for(int j = 0;j < (iDataSetSize+1);j++)
			{
				for(int k = 0;k < (iDataSetSize+1);k++)
				{
					float dis = sqrt((float)((i-iDataSetSize/2)*(i-iDataSetSize/2)+(j-iDataSetSize/2)*(j-iDataSetSize/2)+(k-iDataSetSize/2)*(k-iDataSetSize/2)));
					data[k*(iDataSetSize+1)*(iDataSetSize+1)+j*(iDataSetSize+1)+i] = dis;
				}
			}
		}
		m_MC->SetData(data);
		m_MC->SetTargetValue(fTargetValue);
		m_MC->GenTriangles();


        glutInit(&argc, argv);
        glutInitWindowPosition( 0, 0);
        glutInitWindowSize(iWidth, iHeight);
        glutInitDisplayMode( GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE );
        glutCreateWindow( "Marching Cubes" );
        glutDisplayFunc( vDrawScene );
        glutIdleFunc( vIdle );
        glutReshapeFunc( vResize );
        glutKeyboardFunc( vKeyboard );
        glutSpecialFunc( vSpecial );

        glClearColor( 0.0, 0.0, 0.0, 1.0 ); 
        glClearDepth( 1.0 ); 

        glEnable(GL_DEPTH_TEST); 
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, ePolygonMode);

        glLightfv( GL_LIGHT0, GL_AMBIENT,  afPropertiesAmbient); 
        glLightfv( GL_LIGHT0, GL_DIFFUSE,  afPropertiesDiffuse); 
        glLightfv( GL_LIGHT0, GL_SPECULAR, afPropertiesSpecular); 
        glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 1.0); 

        glEnable( GL_LIGHT0 ); 

        glMaterialfv(GL_BACK,  GL_AMBIENT,   afAmbientGreen); 
        glMaterialfv(GL_BACK,  GL_DIFFUSE,   afDiffuseGreen); 
        glMaterialfv(GL_FRONT, GL_AMBIENT,   afAmbientBlue); 
        glMaterialfv(GL_FRONT, GL_DIFFUSE,   afDiffuseBlue); 
        glMaterialfv(GL_FRONT, GL_SPECULAR,  afSpecularWhite); 
        glMaterialf( GL_FRONT, GL_SHININESS, 25.0); 

        vResize(iWidth, iHeight); 

        vPrintHelp();
        glutMainLoop(); 
}

void vPrintHelp()
{
        printf("Marching Cubes Example by Cory Bloyd (dejaspaminacan@my-deja.com)\n\n");

        printf("+/-  increase/decrease sample density\n");
        printf("PageUp/PageDown  increase/decrease surface value\n");
        printf("s  change sample function\n");
        printf("c  toggle marching cubes / marching tetrahedrons\n");
        printf("w  wireframe on/off\n");
        printf("l  toggle lighting / color-by-normal\n");
        printf("Home  spin scene on/off\n");
        printf("End  source point animation on/off\n");
}


void vResize( GLsizei iWidth, GLsizei iHeight ) 
{ 
        float fAspect, fHalfWorldSize = (1.4142135623730950488016887242097/2); 

        glViewport( 0, 0, iWidth, iHeight ); 
        glMatrixMode (GL_PROJECTION);
        glLoadIdentity ();

        if(iWidth <= iHeight)
        {
                fAspect = (float)iHeight / (float)iWidth;
                glOrtho(-fHalfWorldSize, fHalfWorldSize, -fHalfWorldSize*fAspect,
                        fHalfWorldSize*fAspect, -10*fHalfWorldSize, 10*fHalfWorldSize);
        }
        else
        {
                fAspect = (float)iWidth / (float)iHeight; 
                glOrtho(-fHalfWorldSize*fAspect, fHalfWorldSize*fAspect, -fHalfWorldSize,
                        fHalfWorldSize, -10*fHalfWorldSize, 10*fHalfWorldSize);
        }
 
        glMatrixMode( GL_MODELVIEW );
}

void vKeyboard(unsigned char cKey, int iX, int iY)
{
        switch(cKey)
        {
				case 't' :
				{
					fTargetValue /= 2.0;
					m_MC->SetTargetValue(fTargetValue);
					m_MC->GenTriangles();
				}break;
				case 'y' :
				{
					fTargetValue *= 2.0;
					m_MC->SetTargetValue(fTargetValue);
					m_MC->GenTriangles();
				}break;
                case 'w' :
                {
                        if(ePolygonMode == GL_LINE)
                        {
                                ePolygonMode = GL_FILL;
                        }
                        else
                        {
                                ePolygonMode = GL_LINE;
                        }
                        glPolygonMode(GL_FRONT_AND_BACK, ePolygonMode);
                } break;
                /*case '+' :
                case '=' :
                {
                        ++iDataSetSize;
                        fStepSize = 1.0/iDataSetSize;
                } break;
                case '-' :
                {
                        if(iDataSetSize > 1)
                        {
                                --iDataSetSize;
                                fStepSize = 1.0/iDataSetSize;
                        }
                } break;*/
                //case 'c' :
                //{
                //        if(vMarchCube == vMarchCube1)
                //        {
                //                vMarchCube = vMarchCube2;//Use Marching Tetrahedrons
                //        }
                //        else
                //        {
                //                vMarchCube = vMarchCube1;//Use Marching Cubes
                //        }
                //} break;
                /*case 's' :
                {
                        if(fSample == fSample1)
                        {
                                fSample = fSample2;
                        }
                        else if(fSample == fSample2)
                        {
                                fSample = fSample3;
                        }
                        else if(fSample == fSample3)
                        {
                                fSample = fSample4;
                        }
						else
						{
								fSample = fSample1;
						}
                } break;*/
                case 'l' :
                {
                        if(bLight)
                        {
                                glDisable(GL_LIGHTING);//use vertex colors
                        }
                        else
                        {
                                glEnable(GL_LIGHTING);//use lit material color
                        }

                        bLight = !bLight;
                };
        }
}


void vSpecial(int iKey, int iX, int iY)
{
        switch(iKey)
        {
                case GLUT_KEY_PAGE_UP :
                {
                        if(fTargetValue < 1000.0)
                        {
                                fTargetValue *= 1.1;
                        }
                } break;
                case GLUT_KEY_PAGE_DOWN :
                {
                        if(fTargetValue > 1.0)
                        {
                                fTargetValue /= 1.1;
                        }
                } break;
                case GLUT_KEY_HOME :
                {
                        bSpin = !bSpin;
                } break;
                case GLUT_KEY_END :
                {
                        bMove = !bMove;
                } break;
        }
}

void vIdle()
{
        glutPostRedisplay();
}

void vDrawScene() 
{ 
        static float fPitch = 0.0;
        static float fYaw   = 0.0;
        static float fTime = 0.0;

        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ); 

        glPushMatrix(); 

        if(bSpin)
        {
                fPitch += 4.0;
                fYaw   += 2.5;
        }
        if(bMove)
        {
                fTime  += 0.025;
        }

        vSetTime(fTime);

        glTranslatef(0.0, 0.0, -1.0);  
        glRotatef( -fPitch, 1.0, 0.0, 0.0);
        glRotatef(     0.0, 0.0, 1.0, 0.0);
        glRotatef(    fYaw, 0.0, 0.0, 1.0);

        glPushAttrib(GL_LIGHTING_BIT);
                glDisable(GL_LIGHTING);
                glColor3f(1.0, 1.0, 1.0);
                glutWireCube(1.0); 
        glPopAttrib(); 


        glPushMatrix(); 
        glTranslatef(-0.5, -0.5, -0.5);
		
        glBegin(GL_TRIANGLES);
			int num = m_MC->GetNumOfTriangle();
			//printf("num = %d\n",num);
			for(int i = 0;i < num;i++)
			{
				MarchingCubeBase::MCtriangle trian = m_MC->GetTriangle(i);
				for(int j = 0;j < 3; j++)
				{
					glColor3f(trian.v[j].color.x,trian.v[j].color.y,trian.v[j].color.z);
					glNormal3f(trian.v[j].normal.x,trian.v[j].normal.y,trian.v[j].normal.z);
					glVertex3f((trian.v[j].v.x + 0.5) / iDataSetSize, trian.v[j].v.y / iDataSetSize, trian.v[j].v.z / iDataSetSize);
				}
			}
			
        glEnd();
        glPopMatrix(); 


        glPopMatrix(); 

        glutSwapBuffers(); 
}








//Generate a sample data set.  fSample1(), fSample2() and fSample3() define three scalar fields whose
// values vary by the X,Y and Z coordinates and by the fTime value set by vSetTime()
void vSetTime(float fNewTime)
{
        float fOffset;
        int iSourceNum;

        for(iSourceNum = 0; iSourceNum < 3; iSourceNum++)
        {
                sSourcePoint[iSourceNum].x = 0.5;
                sSourcePoint[iSourceNum].y = 0.5;
                sSourcePoint[iSourceNum].z = 0.5;
        }

        fTime = fNewTime;
        fOffset = 1.0 + sinf(fTime);
        sSourcePoint[0].x *= fOffset;
        sSourcePoint[1].y *= fOffset;
        sSourcePoint[2].z *= fOffset;
}



