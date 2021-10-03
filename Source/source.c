#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <GL/glfw.h>
#include <FTGL/ftgl.h>

#define CrossingNumber 100  /* The number of crossings = 100 */
#define MaxName       50
#define PI 3.14159265

#define GLFW_KEY_Z 90
#define GLFW_KEY_X 88
#define GLFW_KEY_W 87
#define GLFW_KEY_S 83
#define GLFW_KEY_D 68
#define GLFW_KEY_A 65
#define GLFW_KEY_Q 81
#define GLFW_KEY_E 69
#define GLFW_KEY_R 82

#define GLFW_MOUSE_BUTTON_1 0
#define GLFW_MOUSE_NUTTON_2 1

#define GLFW_MOUSE_BUTTON_LEFT  GLFW_MOUSE_BUTTON_1
#define GLFW_MOUSE_BUTTON_RIGHT GLFW_MOUSE_BUTTON_2


#define PathNumber 100
#define Radius_Marker 0.2

double ORIGIN_X = 0.0;
double ORIGIN_Y = 0.0;
double REAL_SIZE_X = 20.0;
double REAL_SIZE_Y = 20.0;
float zoom = 0;

double  z_deg = 0.0;
double currentangle = 0.0;
double incrementangle = 0.0;
double prev_angle = 0.0;
double vehicle_x = 0.0;
double vehicle_y = 0.0;
int rotationflag = 1;
int nav_mode = 0;
int current_state = -1;
int prev_state = -1;
int prev2_state = -1;
int vehicle_start = -1;
int vehicle_goal = -1;
int clicked_start = 0;
int clicked_goal = 0;



typedef struct
{
  double x, y;       /* Position x, y */
} Position;                /* A structure that represents positions */ 
 
typedef struct
{
  int id;               /* Crossing number */
  Position pos;           /* A structure that represents positions */
  double wait;          /* Average waiting time */
  char  jname[MaxName];  /* Crossing name (Japanese) */
  char ename[MaxName];  /* Crossing name (Alphabetical name) */
  int points;            /* The number of crossing roads */
  int next[5]; /* Crossing numbers of neighboring crossings */
  double d;
  int nextstep;
  int done;
} Crossing;

Crossing cross[CrossingNumber];
int path[PathNumber +1];

void circle(double x, double y, double r)
{
  int const N = 12;             /* Divide the circumference by 12 and draw in line segments */
  int i;
  glBegin(GL_LINE_LOOP);
  for (i = 0; i < N; i++)
    glVertex2d(x + cos(2 * M_PI * i / N) * r, y + sin(2 * M_PI * i / N) * r);
  glEnd();
}

void arrow (double x, double y, double r)
{
            
  
  glBegin(GL_LINE_LOOP);
  glVertex2d(x , y + r);
  glVertex2d(x+(r*sin(PI/3)) , y -(r*cos(PI/3)));
  glVertex2d(x , y );
  glVertex2d(x-(r*sin(PI/3)) , y -(r*cos(PI/3)));	     
  glEnd();
     
    
  
}

#ifndef FONT_FILENAME
//#define FONT_FILENAME "/usr/share/fonts/truetype/fonts-japanese-gothic.ttf"
#define FONT_FILENAME "/usr/share/fonts/truetype/gentium-basic/GenBkBasB.ttf"
#endif
FTGLfont *font; 

void outtextxy(double x, double y, char const *text)
{
  double const scale = 0.01;
  glPushMatrix();
  glTranslated(x, y, 0.0); 
  glScalef(REAL_SIZE_X/12, REAL_SIZE_X/12, REAL_SIZE_X/12);
    
  if(nav_mode == 1)
    {
      if(rotationflag == 1)
	{
     
	  if(fabs(z_deg) >= fabs(5*incrementangle))
	    {
	      glRotatef(-currentangle, 0.0, 0.0, 1.0);
	    }
	  else
	    {
	      glRotatef(-(prev_angle+z_deg), 0.0, 0.0, 1.0);
	    }
	}
      else
	{
	  glRotatef(-currentangle, 0.0, 0.0, 1.0); 
	}
    }
  glScaled(scale, scale, scale);
  ftglRenderFont(font, text, FTGL_RENDER_ALL);
  glPopMatrix();
}

double distance(int a, int b)
{
  return hypot(cross[a].pos.x-cross[b].pos.x, cross[a].pos.y-cross[b].pos.y);
}

void dijkstra (int target)
{
  int a=target;
  int i,j,k;
  double mdist=1000000000;
  for(i=0;i<CrossingNumber;i++)
    {
      cross[i].d=100000000000.00;
      cross[i].done = 0;
    }
  cross[target].d = 0;
  cross[target].nextstep = -1;
  
  for(j=0;j<CrossingNumber;j++)
    {
      if(cross[a].done<1)
	{
	  for(i=0;i<cross[a].points;i++)
	    {
	      k=cross[a].next[i];  
	      if(distance(a, k)+cross[a].d < cross[k].d)
		{
		  cross[k].d =cross[a].d + distance(a, k);
		  cross[k].nextstep = a;
		}
	    }
	  cross[a].done = 1;
	  mdist=100000000000000;
	}
      for(i=0;i<CrossingNumber;i++)
	{
	  if(cross[i].done<1)
	    {
	      if(cross[i].d < mdist)
		{
		  mdist=cross[i].d;
		  a = i;
		}
	     
	    }
	}	   
    }
}

int map_read(char *filename)
{
  FILE *fp;
  int i, j;
  int crossing_number;                         /* Number of intersections */

  fp = fopen(filename, "r");

  if (fp == NULL)
    {
      printf("File %s is not created\n", filename);
      return 0;
    }
  /* First read the number of intersections */
  fscanf(fp, "%d", &crossing_number);
  if((crossing_number<1)||     (crossing_number>=CrossingNumber))
    {
      printf("Illegal data number (%d)\n", crossing_number);
      return 0;
    }
  for (i = 0; i < crossing_number; i++)
    {
      fscanf(fp, "%d,%lf,%lf,%lf,%[^,],%[^,],%d",       &(cross[i].id), &(cross[i].pos.x), &(cross[i].pos.y),       &(cross[i].wait), cross[i].jname, cross[i].ename,        &(cross[i].points));

      for(j=0; j < cross[i].points; j++)
	{
	  fscanf(fp, ",%d", &(cross[i].next[j]));
	}
    }
  fclose(fp);
 
  /* Return the number of intersections read from the file */
  return crossing_number;
}

void line(double x0, double y0, double x1, double y1)
{
  glBegin(GL_LINES);
  glVertex2d(x0, y0);
  glVertex2d(x1, y1);
  glEnd();
}

void map_show (int crossing_number)
{
  int i,j;

  for(i=0;i<crossing_number;i++)
    {
      glColor3d(1.0, 0.0, 0.0);
      if(nav_mode == 1)
	{
	  circle(cross[i].pos.x, cross[i].pos.y,0.02);
	}
      else
	{
	  circle(cross[i].pos.x, cross[i].pos.y,(0.1/20)*REAL_SIZE_X);
	}
      
      glColor3d(1.0, 1.0, 1.0);
      glBegin(GL_LINES);
      for(j=0;j<cross[i].points;j++)
	{
	  
	  glVertex2d(cross[i].pos.x, cross[i].pos.y);
	  glVertex2d(cross[cross[i].next[j]].pos.x, cross[cross[i].next[j]].pos.y);
	}
      glEnd();
    }
}

void text_show(int crossing_number, int vehicle_start, int vehicle_goal)
{
  int i;
  int from;
  int to;
  from = vehicle_start;
  if(nav_mode == 0)
    {
      for(i=0;i<crossing_number;i++)
	{
	  glColor3d(1.0, 1.0, 0.0);
	  outtextxy(cross[i].pos.x, cross[i].pos.y, cross[i].ename);
	}
    }
  else
    {
      glColor3d(1.0, 1.0, 0.0);
      outtextxy(cross[from].pos.x, cross[from].pos.y, cross[from].ename);
      do
	{
	  to = cross[from].nextstep;
	  glColor3d(1.0, 1.0, 0.0);
	  outtextxy(cross[to].pos.x, cross[to].pos.y, cross[to].ename);
	  from = to;
	}while(to!=vehicle_goal);
    }
  
}

void z_rotation(int deg, double xloc, double yloc)
{
  glTranslatef(xloc, yloc, 0);
  glRotatef(deg, 0.0, 0.0, 1.0);
  glTranslatef(-xloc, -yloc, 0);
}

void startgoal_show(int start, int goal)
{
  int from;
  int to;
  from = start;
  glColor3f(1.0, 0.5, 0.0);
  if(nav_mode == 1)
    {
      circle(cross[start].pos.x, cross[start].pos.y, 0.02);
      circle(cross[goal].pos.x, cross[goal].pos.y, 0.02);
    }
  else
    {
      circle(cross[start].pos.x, cross[start].pos.y, (0.1/20)*REAL_SIZE_X);
      circle(cross[goal].pos.x, cross[goal].pos.y, (0.1/20)*REAL_SIZE_X);
    }
  
  glBegin(GL_LINES);
  do
    {
      to = cross[from].nextstep;
      glVertex2d(cross[from].pos.x, cross[from].pos.y);
      glVertex2d(cross[to].pos.x, cross[to].pos.y);
      from = to;
    }while(to!=goal);
  glEnd();
}

int anglerot(int from, int to)
{
  double del_x, del_y, tan, ret, val;
  
  del_x = (cross[to].pos.x)-(cross[from].pos.x);
  del_y = (cross[to].pos.y)-(cross[from].pos.y);
  
  tan = (del_y/del_x);
  val = 180.0 / PI;
  
  if(del_x>=0)
    {
      ret = 90 - (atan (tan) * val);
    }
  else
    {
      ret = -90 - (atan (tan) * val);
    }
  
  return ret;
}

void cross_rotate(double xloc, double yloc)
{
  if(rotationflag == 1)
    { 
      if(fabs(z_deg) >= fabs(5*incrementangle))
	{
	  rotationflag=0;
	  z_rotation(currentangle, xloc, yloc);
	  
	}
      else
	{
	  z_rotation(prev_angle + z_deg, xloc, yloc); 
	}	
    } 
  else
    {
      z_rotation(currentangle, xloc, yloc); 
    }
}
	


double incre_angle (int cross_id)
{
  double theta_1 = anglerot(cross_id, cross[cross_id].nextstep);
  double theta_2 = anglerot(cross[cross_id].nextstep, cross[cross[cross_id].nextstep].nextstep);
  double ret = 0;
  
  if(theta_1 > 0 && theta_2 > 0)
    {
      ret = theta_2 - theta_1;
    }
  else if (theta_1 < 0 && theta_2 < 0)
    {
      ret = theta_2 - theta_1;
    }
  else if (theta_1 > 0 && theta_1 < 90 && theta_2 < 0 && theta_2 > -90)
    {
      ret = theta_2 - theta_1;
    }
  else if (theta_1 >0 && theta_2<0 )
    {
      if(theta_2 < -(180-theta_1))
	{
    
	  ret = 360 + theta_2 - theta_1;
	}
      else
	{
	  ret = theta_2 - theta_1;
	}
    }
  else if (theta_2>0 && theta_1<0)
    {
      if(theta_2 > (180 + theta_1))
	{
	  ret = -360 + theta_2 - theta_1;
	}
      else
	{
	  ret = theta_2 - theta_1;
	}
    }
  else
    {
      ret = theta_2 - theta_1;
    }

  return ret;
  
}

void get_real_coordinates(double x, double y, double* real_x, double* real_y)
{
  *real_x = ORIGIN_X + (x-500)*REAL_SIZE_X/1000;
  *real_y = ORIGIN_Y + (400-y)*REAL_SIZE_Y/800;
}


void green_circle(double real_x, double real_y, int crossing_number)
{
  double r[crossing_number];
  int i;
  int click;

  
  
  for(i=0;i<crossing_number;i++)
    {
      r[i]=hypot(real_x-cross[i].pos.x, real_y-cross[i].pos.y);
     

      if(r[i]<(REAL_SIZE_X/20.0)*0.3)
	{
	  glColor3f(0.0, 1.0, 0.0);
	  circle(cross[i].pos.x, cross[i].pos.y, (REAL_SIZE_X/20)*0.2);
	  click = mouse_click();
	  if(click == 1)
	    {
	      
	      if(i == vehicle_start)
		{
		  clicked_start = 0;
		}
	      else if(i == vehicle_goal)
		{
		  clicked_goal = 0;
		}
	      else if(clicked_start == 0)
		{
		  glColor3f(1.0, 0.5, 0.0);
		  circle(cross[i].pos.x, cross[i].pos.y, (REAL_SIZE_X/20)*0.3);
		  vehicle_start = i;
		  clicked_start = 1;
		}
	      else if(clicked_goal == 0)
		{
		  glColor3f(1.0, 0.5, 1.0);
		  circle(cross[i].pos.x, cross[i].pos.y, (REAL_SIZE_X/20)*0.3);
		  vehicle_goal = i;
		  clicked_goal = 1;
		  dijkstra (vehicle_goal);
		  vehicle_x = cross[vehicle_start].pos.x;
		  vehicle_y = cross[vehicle_start].pos.y;
		}
	      
	    }	  
	}
    }
}

int mouse_click()
{
  
  int click = 0;

  prev2_state = prev_state;
  prev_state = current_state;
  current_state = glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT);
  
  if(current_state == GLFW_RELEASE && prev_state == GLFW_PRESS && prev2_state == GLFW_RELEASE)
    {
      click = 1;
    }
  return click;
}

void search_cross(int crossing_number)
{
  int i,j;
  char buff[256];
 int f=0;
  printf("Input the crossing name (in English)>");
  scanf("%s",buff);
    

  for(i=0;i<crossing_number;i++)
    {
      if(strcmp(cross[i].ename,buff)==0)
	{


	  printf("%d, (%lf, %lf), %s, %s", cross[i].id, cross[i].pos.x, cross[i].pos.y, cross[i].ename, cross[i].jname);
	  printf(" %lf, %d,", cross[i].wait, cross[i].points);

	  for(j=0; j<cross[i].points; j++)
	    {
	      printf("%d ", cross[i].next[j]);
	    }
	  printf("\n");
	  f=1;
	  ORIGIN_X = cross[i].pos.x;
	  ORIGIN_Y = cross[i].pos.y;
	    
	}
    }
  if(f==0) search_cross_approx(crossing_number, buff);
}

void search_cross_approx(int crossing_number, char buff[256])
{
  int i,j;
    
  int f=0;
   

  for(i=0;i<crossing_number;i++)
    {
      if(strstr(cross[i].ename,buff)!=NULL)
	{
	  printf("%d, (%lf, %lf), %s, %s", cross[i].id, cross[i].pos.x, cross[i].pos.y, cross[i].ename, cross[i].jname);
	  printf(" %lf, %d,", cross[i].wait, cross[i].points);

	  for(j=0; j<cross[i].points; j++)
	    {
	      printf("%d ", cross[i].next[j]);
	    }
	  printf("\n");
	  f=1;
	}
    }
  if(f==0) puts("The data has not been found");
}

void search_min_distance(int crossing_number)
{
  int i,j;
  int mini_index=0;
  double dist, mdist;
  double x,y;

  printf("Input your position>\n");
  printf("x-position>");
  scanf("%lf",&x);
  printf("y-position>");
  scanf("%lf",&y);

  for(i=0;i<crossing_number;i++)
    {
      mdist = hypot(cross[mini_index].pos.x-x, cross[mini_index].pos.y-y);
      dist = hypot(cross[i].pos.x-x, cross[i].pos.y-y);

      if(dist<mdist)
	{
	  mini_index=i;
	}
    }
  printf("Crossing ID, (x-location, y-location), English Name, Japanese Name, Waiting time, Number of Convergence Road, Nearest Crossing\n");

  printf("%d, (%lf, %lf), %s, %s", cross[mini_index].id, cross[mini_index].pos.x, cross[mini_index].pos.y, cross[mini_index].ename, cross[mini_index].jname);
  printf(" %lf, %d,", cross[mini_index].wait, cross[i].points);

  for(j=0; j<cross[mini_index].points; j++)
    {
      printf("%d ", cross[mini_index].next[j]);
    }
  printf("\n");
  printf("Distance : %.2lf\n",hypot(cross[i].pos.x-x,cross[i].pos.y-y));
}


int main(void)
{
  int crossing_number;
  int vehicle_edgeFrom = -1;
  int vehicle_edgeTo;
  int vehicle_stepOnEdge = 0;
  int mouse_x, mouse_y;
  double real_x, real_y;
 
  
  
  /* Initialize the graphic environment and open a window */
  glfwInit();
  glfwOpenWindow(1000, 800, 0, 0, 0, 0, 0, 0, GLFW_WINDOW); 
 
  /* Based on (ORIGIN_X, ORIGIN_Y), project the space within the range of       REAL_SIZE_X * REAL_SIZE_Y to the viewport */
  
  /* Read and configure the font used for drawing strings */
  font = ftglCreateExtrudeFont(FONT_FILENAME);
  if (font == NULL)
    {
      perror(FONT_FILENAME);
      fprintf(stderr, "could not load font\n");
      exit(1);
    }
  ftglSetFontFaceSize(font, 24, 24);
  ftglSetFontDepth(font, 0.01);
  ftglSetFontOutset(font, 0, 0.1);
  ftglSetFontCharMap(font, ft_encoding_unicode); 
 
  /* Read the map file */
  crossing_number = map_read("map.dat");
  if (crossing_number < 0)
    {
      fprintf(stderr, "couldn't read map file\n");
      exit(1);
    }

  printf("Intruction Manual\n");
  printf("* To zoom in or out, use mouse scrolling                                         *\n");
  printf("* To move to the north, press 'w'                                                *\n");
  printf("* To move to the south, press 's'                                                *\n");
  printf("* To move to the east, press 'd'                                                 *\n");
  printf("* To move to the west, press 'a'                                                 *\n");
  printf("* To search crossing, press 'e'                                                  *\n");
  printf("* If more than one crossing appear, press 'e' again and input the correct name   *\n");
  printf("* To choose start and goal, use mouse left button, and click at the map location *\n");
  printf("* Start is shown as brown circle, goal is shown as pink circle                   *\n");
  printf("* To start navigation mode, press 'enter'                                        *\n");
  printf("Crossing ID, (x-location, y-location), English Name, Japanese Name, Waiting time, Number of Convergence Road, Nearest Crossing\n");
 

 
  do{
    
      do
	{
	  int width, height;
      
 
	  // End when Esc is pressed or the window is closed
      
	  if (!glfwGetWindowParam(GLFW_OPENED) || glfwGetKey(GLFW_KEY_ESC))
	    break; 
 
       
       
	  glfwGetWindowSize(&width, &height);  // Get the size of the current window
	  glViewport(0, 0, width, height);  // Set the whole window as a viewport 
 
	  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	  glClear(GL_COLOR_BUFFER_BIT);  // Fill the back buffer with black 
 
      
     
	  glMatrixMode(GL_PROJECTION);
	  glLoadIdentity();
	  glOrtho(ORIGIN_X + REAL_SIZE_X * -0.5, ORIGIN_X + REAL_SIZE_X * 0.5,           ORIGIN_Y + REAL_SIZE_Y * -0.5, ORIGIN_Y + REAL_SIZE_Y * 0.5,           -1.0, 1.0); 
 
	  glMatrixMode(GL_MODELVIEW);
	  glLoadIdentity();             // Do not perform other coordinate conversion 
      
	  map_show(crossing_number);
	  text_show(crossing_number, vehicle_start, vehicle_goal);
     
      
	  glfwWaitEvents();
	  glfwGetMousePos(&mouse_x, &mouse_y);
	  get_real_coordinates(mouse_x, mouse_y, &real_x, &real_y);
	  green_circle(real_x, real_y, crossing_number);

	  if(glfwGetKey(GLFW_KEY_ESC))
	    {
	      break;
	    }

	  if(clicked_start == 1)
	    {
	      glColor3f(1.0, 0.5, 0.0);
	      circle(cross[vehicle_start].pos.x, cross[vehicle_start].pos.y, (REAL_SIZE_X/20)*0.3);
	    }
	  if(clicked_goal == 1)
	    {
	      glColor3f(1.0, 0.5, 1.0);
	      circle(cross[vehicle_goal].pos.x, cross[vehicle_goal].pos.y, (REAL_SIZE_X/20)*0.3);
	    }

	  if(clicked_goal==1 && clicked_start ==1){
	    vehicle_edgeFrom = vehicle_start ;
	    startgoal_show(vehicle_start, vehicle_goal);
	  } 
     
	  zoom = glfwGetMouseWheel();
	  if(zoom < 60)
	    {
	      REAL_SIZE_X = 20.0*((60-zoom)/60);
	      REAL_SIZE_Y = 20.0*((60-zoom)/60);
	    }
                  
	  if (glfwGetKey(GLFW_KEY_ENTER))
	    {
	      nav_mode = 1;
	      break;
	    }
      
	  if(glfwGetKey(GLFW_KEY_W))
	    {
	      ORIGIN_Y += (0.5/20)*REAL_SIZE_X;
	    }
	  if(glfwGetKey(GLFW_KEY_S))
	    {
	      ORIGIN_Y -= (0.5/20)*REAL_SIZE_X;
	    }
	  if(glfwGetKey(GLFW_KEY_D))
	    {
	      ORIGIN_X += (0.5/20)*REAL_SIZE_X;
	    }
	  if(glfwGetKey(GLFW_KEY_A))
	    {
	      ORIGIN_X -= (0.5/20)*REAL_SIZE_X;
	    }
	  if(glfwGetKey(GLFW_KEY_E))
	    {
	      search_cross(crossing_number);
	    } 
   
       

	  glfwSwapBuffers();
      

	  usleep(0.001*100);
	} while(nav_mode == 0);
      do
	{
	  int width, height;
	  REAL_SIZE_X = 2.0;
	  REAL_SIZE_Y = 2.0;
 
	  /* End when Esc is pressed or the window is closed */
      
	  if (glfwGetKey(GLFW_KEY_ESC) || !glfwGetWindowParam(GLFW_OPENED))
	    break;
	  if( vehicle_x == cross[vehicle_goal].pos.x && vehicle_y == cross[vehicle_goal].pos.y)
	    {
	      nav_mode = 0;
	      break;
	    }
 
	  glfwGetWindowSize(&width, &height);  /* Get the size of the current window */
	  glViewport(0, 0, width, height);  /* Set the whole window as a viewport */ 
 
	  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	  glClear(GL_COLOR_BUFFER_BIT);  /* Fill the back buffer with black */ 
 
      
	  ORIGIN_X = vehicle_x;
	  ORIGIN_Y = vehicle_y;
	  glMatrixMode(GL_PROJECTION);
	  glLoadIdentity();
	  glOrtho(ORIGIN_X + REAL_SIZE_X * -0.5, ORIGIN_X + REAL_SIZE_X * 0.5,           ORIGIN_Y + REAL_SIZE_Y * -0.5, ORIGIN_Y + REAL_SIZE_Y * 0.5,           -1.0, 1.0); 
 
	  glMatrixMode(GL_MODELVIEW);
	  glLoadIdentity();             /* Do not perform other coordinate conversion */ 

            
	  currentangle = anglerot(vehicle_edgeFrom, cross[vehicle_edgeFrom].nextstep);
      
      
	  cross_rotate(vehicle_x, vehicle_y);
	  map_show(crossing_number);
	  text_show(crossing_number, vehicle_start, vehicle_goal);
	  if(rotationflag ==1)
	    {
	      z_deg += incrementangle;
	    }
	  else
	    {
	      z_deg = 0;
	    }  
      
	  startgoal_show(vehicle_start, vehicle_goal);
     
          
      
	  vehicle_edgeTo = cross[vehicle_edgeFrom].nextstep;

      
	  if(vehicle_edgeFrom != vehicle_goal)
	    {
	      double x0 = cross[vehicle_edgeFrom].pos.x;
	      double y0 = cross[vehicle_edgeFrom].pos.y;
	      double x1 = cross[vehicle_edgeTo].pos.x;
	      double y1 = cross[vehicle_edgeTo].pos.y;

	      double distance = hypot(x1-x0, y1-y0);
	      int steps = (int)(distance/0.1);
	  
	      if(rotationflag == 0)
		{
		  vehicle_stepOnEdge ++;
		}
	      else
		{
		  vehicle_stepOnEdge = 0;
		}
	      vehicle_x = x0 + ((x1-x0)/steps)*vehicle_stepOnEdge;
	      vehicle_y = y0 + ((y1-y0)/steps)*vehicle_stepOnEdge;
	  
	      if(vehicle_stepOnEdge>= steps)
		{
		  prev_angle = currentangle;
		  incrementangle = incre_angle(vehicle_edgeFrom)/5.0;
		  rotationflag = 1;
		  vehicle_edgeFrom = vehicle_edgeTo;
		  vehicle_stepOnEdge =0;
		}
	    }
	  if(rotationflag==1)
	    {
	      if(fabs(z_deg) >= fabs(5*incrementangle))
		{
		  z_rotation(-currentangle, vehicle_x, vehicle_y);
		}
	      else
		{
		  z_rotation(-(prev_angle+z_deg), vehicle_x, vehicle_y);
		}
	    }
	  else
	    {
	      z_rotation(-currentangle, vehicle_x, vehicle_y);
	    }
	  
	  glColor3f(0.0, 4.0, 0.0);
	  arrow(vehicle_x, vehicle_y, 0.07);
	  
	  glfwSwapBuffers();      
	  usleep(2000*100);
	  
	}while(nav_mode == 1);
  }while(!glfwGetKey(GLFW_KEY_ESC));
      // goto start;
  // goto start;
  
  glfwTerminate( ); 
  
  return 0;
} 

//compilecode (gcc source.c -g -O2 -Wall -I/usr/include/freetype2 -lftgl -lglfw -lGLU -lGL -lX11 -lXrandr -lm -o executable)
