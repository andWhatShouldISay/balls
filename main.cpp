#include <GL/glut.h>
#include "SOIL.h"
#include "dsu.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <random>
#include <cassert>
#include <map>
#include <ctime>
#include <set>

#define unpair(p) (p).first][(p).second

using namespace std;

int width;
int height;

const double real_table_length=190;
const double real_ball_diameter=5.25;

unsigned int window;

map<unsigned int,unsigned int> texid;

struct circle
{
    pair<int,int> coord;
    int r;
    int step;
    vector<int> RGB;


    const bool operator<(const circle& c)
    {
        if (step!=c.step)
            return step<c.step;
        if (coord!=c.coord)
            return coord<c.coord;
        return r<c.r;
    }

};

vector<circle> circles;

struct table
{
    int yu,yd;
    int xul,xur,xdl,xdr;
    int ymid;

    table():yu(-1),yd(-1),xul(-1),xur(-1),xdl(-1),xdr(-1) {}

    double leftX(int y)
    {
        const static int height = yd-yu;
        double dy=y-yu;
        int dxl=xul-xdl;
        int xl=xul-dxl*dy/height;

        return xl;
    }

    double rightX(int y)
    {
        const static int height = yd-yu;
        double dy=y-yu;
        int dxr=xdr-xur;
        int xr=xur+dxr*dy/height;

        return xr;

    }

    bool in(int y,int x)
    {
        const static int height = yd-yu;
        if (yu<=y && y<=yd)
        {
            return leftX(y) <= x && x <= rightX(y);

        }
        return 0;
    }

    vector<pair<int,int> > iterate()
    {
        vector<pair<int,int> > v;
        for (int i=yu; i<=yd; i++)
            for (int j=xdl; j<=xdr; j++)
                if (in(i,j))
                    v.push_back({i,j});
        return v;
    }

    double close_side()
    {
        return xdr-xdl;
    }
    double far_side()
    {
        return xur-xul;
    }

    double lenAt(int y)
    {
        return rightX(y)-leftX(y);
    }


    double multiplier(int y)
    {
        /* yd -> close_side/real_length
           yu -> far_side/real_length
        */

        double k=(close_side()/real_table_length-far_side()/real_table_length)/(yd-yu);

        return (y-yd)*k+close_side()/real_table_length;

    }

    double realX(int y,int x,const int table_width)
    {
        int l=x-leftX(y);
        int d=rightX(y)-leftX(y);

        return (real_ball_diameter+l*1.0*real_table_length/d)/(2*real_ball_diameter+real_table_length)*table_width;
    }

    double realY(int y,int x,const int table_width,const int table_height)
    {
        //yd -> 0
        //yu -> table height

        const static double a=1000/close_side();
        const static double b=1000/far_side();
        const static double m=1000/lenAt(ymid);

        const static double ang1=acos((a*a+4*m*m-b*b)/(4*a*m));
        const static double ang2=acos((b*b+4*m*m-a*a)/(4*b*m));
        const static double ang=ang1+ang2;


        const static double image_height=sqrt(a*a+b*b-2*a*b*cos(ang));


        const static double ang_close=acos(-1)-asin(sin(ang)*b/image_height);

        double len=1000/lenAt(y);


        double ang_here = asin(a*sin(ang_close)/len);

        double ang_camera = acos(-1)-ang_here-ang_close;

        double dy = image_height-sqrt(a*a+len*len-2*a*len*cos(ang_camera));

        return (real_ball_diameter+dy/image_height*(2*real_table_length-2*real_ball_diameter))/(2*real_table_length)*table_height;
    }


};
table T;


vector<vector<unsigned char> > Rsrc,Gsrc,Bsrc;

vector<pair<int,int> > pressed;


double texU=0,texD=1,texL=0,texR=1;
void display()
{
    glutSetWindow(window);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    glBindTexture(GL_TEXTURE_2D, texid[window]);

    glEnable(GL_TEXTURE_2D);

    glBegin(GL_QUADS);
    glColor3d(1,1,1);

    glTexCoord2d(texL, texU);
    glVertex2i(0,   0);

    glTexCoord2d(texL, texD);
    glVertex2i(0,   height);

    glTexCoord2d(texR, texD);
    glVertex2i(width, height);

    glTexCoord2d(texR, texU);
    glVertex2i(width, 0);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);

    glLineWidth(3);
    glBegin(GL_LINE_LOOP);
    glColor3d(1,0,0);

    glVertex2i(T.xul,T.yu);
    glVertex2i(T.xdl,T.yd);
    glVertex2i(T.xdr,T.yd);
    glVertex2i(T.xur,T.yu);

    glEnd();

    glBegin(GL_LINES);
    glVertex2d(T.leftX(T.ymid),T.ymid);
    glVertex2d(T.rightX(T.ymid),T.ymid);
    glEnd();


    for (auto &c:circles)
    {
        if (!c.r)
            continue;


        glLineWidth(2);
        glBegin(GL_LINE_LOOP);
        glColor3d(c.RGB[0]/255.0,c.RGB[1]/255.0,c.RGB[2]/255.0);
        const int p=10;
        const double pi=acos(-1);

        int y=c.coord.first;
        int x=c.coord.second;
        int r=c.r;

        for (int i=0; i<p; i++)
        {
            glVertex2d(x+r*cos(2*pi*i/p),y+r*sin(2*pi*i/p));
        }
        glEnd();
    }



    glutSwapBuffers();
}

void keyboard(unsigned char k,int y,int x)
{

}

void initGL(int w, int h)
{
    glViewport(0, 0, w-1, h-1);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, w-1, h-1, 0.0, 0.0, 100.0);

    glMatrixMode(GL_MODELVIEW);

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

double hue(int r,int g,int b)
{
    double mx=max(r,max(g,b));
    double mn=min(r,min(g,b));
    if (mx==mn)
        return NAN;

    if (mx==r&&g>=b)
        return 60*(g-b)/(mx-mn);

    if (mx==r&&g<b)
        return 60*(g-b)/(mx-mn)+360;

    if (mx==g)
        return 60*(b-r)/(mx-mn)+120;

    if (mx==b)
        return 60*(r-g)/(mx-mn)+240;
}

double sat(int r,int g,int b)
{
    int mx=max(r,max(g,b));
    int mn=min(r,min(g,b));
    if (mx==0)
        return 0;
    return 1-mn*1.0/mx;
}

int value(int r,int g,int b)
{
    int mx=max(r,max(g,b));
    return mx;
}

int lightness(int r,int g,int b)
{
    int mx=max(r,max(g,b));
    int mn=min(r,min(g,b));
    return (mx+mn)/2;
}

bool isgreen(int r,int g,int b)
{
    int h=hue(r,g,b);
    double s=sat(r,g,b);
    return 75<=h && h<=170 && s>=0.5;
}

bool isgreen_ball(int r,int g,int b,double avb)
{
    int h=hue(r,g,b);
    double s=sat(r,g,b);
    return 130<=h && h<=180 && s>=0.65 && b >= avb && b>=30;
}

bool isyellow(int r,int g,int b)
{
    int h=hue(r,g,b);
    double s=sat(r,g,b);
    int v=max(r,max(g,b));
    return 40 <= h && h <= 75 && s>=0.4 && v>=150;
}

bool ispink(int r,int g,int b)
{
    int h=hue(r,g,b);
    double s=sat(r,g,b);
    return (330<=h || h<=40) && s<0.6;
}

bool iswhite(int r,int g,int b)
{
    int l=lightness(r,g,b);
    if (l>=220)
        return 1;
    int h=hue(r,g,b);
    double s=sat(r,g,b);
    int v=max(r,max(g,b));

    return 50 <= h && h <= 90 && s <= 0.5 && v >= 200;

}

bool isred(int r,int g,int b)
{
    int h=hue(r,g,b);
    double s=sat(r,g,b);
    int v=max(r,max(g,b));

    return (330 <= h || h <= 30) && s>=0.7 && v>=70;
}

bool isblack(int r,int g,int b)
{
    int v=max(r,max(g,b));

    return v<=50;
}

bool isblue(int r,int g,int b)
{
    int h=hue(r,g,b);
    double s=sat(r,g,b);
    int v=max(r,max(g,b));

    return (180 <= h && h <= 270) && s>=0.7 && v>=100;
}

bool isbrown(int r,int g,int b)
{
    int h=hue(r,g,b);
    double s=sat(r,g,b);
    int v=max(r,max(g,b));

    return (25 <= h && h <= 60) && v>=50 && v<=140 && s >=0.3;
}


void mouse(int button,int state,int x,int y)
{
    if (button==GLUT_MIDDLE_BUTTON)
    {
        if (state==GLUT_DOWN)
        {
            pressed.push_back({y,x});
            cout << (int)Rsrc[y][x] << ' ' << (int)Gsrc[y][x] << ' ' << (int)Bsrc[y][x] << ' ';
            cout << hue(Rsrc[y][x],Gsrc[y][x],Bsrc[y][x]) << endl;
        }
    }
}

#define _rgb vector<vector<unsigned char> >&R,vector<vector<unsigned char> >&G,vector<vector<unsigned char> >&B
void form(_rgb,unsigned char *c)
{
    int t=0;
    for (int i=0; i<height; i++)
    {
        for (int j=0; j<width; j++)
        {
            c[t++]=R[i][j];
            c[t++]=G[i][j];
            c[t++]=B[i][j];
            c[t++]=255;
        }
    }
}

unsigned char* c;

void to_src(_rgb)
{
    R=Rsrc;
    G=Gsrc;
    B=Bsrc;
}


int mask_response(const vector<vector<int> >& L,vector<vector<unsigned char> >& A,int y,int x)
{
    int ans=0;
    for (int i=-1; i<=1; i++)
    {
        for (int j=-1; j<=1; j++)
        {
            ans+=L[1+i][1+j]*A[y+i][x+j];
        }
    }
    return ans;
}

const vector<vector<int> > Ly= {{-1,-2,-1},{0,0,0},{1,2,1}};
const vector<vector<int> > Lx= {{-1,0,1},{-2,0,2},{-1,0,1}};

void sobel(_rgb,vector<vector<unsigned char> >& bw,int border)
{
    double m=0;
    for (int i=1; i+1<height; i++)
    {
        for (int j=1; j+1<width; j++)
        {
            int Gx=mask_response(Lx,bw,i,j);
            int Gy=mask_response(Ly,bw,i,j);
            double grad=hypot(Gx,Gy);
            //int val=(grad>=border)*255;
            m=max(m,grad);
        }
    }

    for (int i=1; i+1<height; i++)
    {
        for (int j=1; j+1<width; j++)
        {
            int Gx=mask_response(Lx,bw,i,j);
            int Gy=mask_response(Ly,bw,i,j);
            double grad=hypot(Gx,Gy)/m;
            int val=255*grad;


            if (val>=border)
            {
                val = 255;
            }
            else
            {
                val = 0;
            }

            R[i][j]=val;

        }
    }

    G=B=R;
}

auto orthgrad(int i,int j,int Gx,int Gy)
{
    //if (!Gx&&!Gy)
    //    return make_pair(make_pair(i,j),make_pair(i,j));

    const static double pi=acos(-1);

    swap(Gx,Gy);
    Gx=-Gx;

    auto ang = atan2(Gy,Gx);
    if (ang<0)
        ang+=2*pi;

    int dx,dy;

    if (ang<pi/8)
        dx=1,dy=0;
    else if (ang<3*pi/8)
        dx=1,dy=1;
    else if (ang<5*pi/8)
        dx=0,dy=1;
    else if (ang<7*pi/8)
        dx=-1,dy=1;
    else if (ang<9*pi/8)
        dx=-1,dy=0;
    else if (ang<11*pi/8)
        dx=-1,dy=-1;
    else if (ang<13*pi/8)
        dx=0,dy=-1;
    else if (ang<15*pi/8)
        dx=1,dy=-1;
    else
        dx=1,dy=0;


    return make_pair(make_pair(i+dy,j+dx),make_pair(i-dy,j-dx));
}

auto hough(vector<vector<unsigned char> >& A,int y1,int y2,int x1,int x2,int mindist,int maxdist,int minangle,int maxangle)
{

    const static  double pi=acos(-1);
    int ang=maxangle-minangle,dist=maxdist-mindist;

    vector<vector<int> > acc(ang,vector<int>(dist,0));

    vector<pair<int,int> > hough_space(ang*dist);

    for (int i=0; i<ang; i++)
        for (int j=0; j<dist; j++)
            hough_space[i*dist+j]= {i,j};

    vector<double> sins(ang),coss(ang);
    for (int i=minangle; i<maxangle; i++)
    {
        sins[i-minangle]=sin(i*pi/180);
        coss[i-minangle]=cos(i*pi/180);
    }

    for (int i=y1; i<y2; i++)
    {
        for (int j=x1; j<x2; j++)
        {
            if (A[i][j])
            {
                for (int k=0; k<ang; k++)
                {
                    int r=(j-x1)*coss[k]+(i-y1)*sins[k];

                    if(mindist<=r && r<maxdist)
                    {
                        ++acc[k][r-mindist];
                    }
                    if (mindist<=r+1 && r+1<maxdist)
                    {
                        ++acc[k][r-mindist+1];
                    }
                }
            }
        }
    }

    sort(hough_space.begin(),hough_space.end(),[&acc](auto a,auto b)
    {
        return acc[unpair(a)]>acc[unpair(b)];
    });

    return hough_space;
}

/*
    rho = x cos(theta) + y cos (theta)
    x = rho / cos(theta) - y tg (thera)
*/
double X(pair<int,int> theta_rho,double y,int minangle)
{
    const static double pi = acos(-1);
    double theta=(theta_rho.first+minangle)*pi/180;

    return theta_rho.second/cos(theta) - y * tan(theta);

}

auto hough_circle(vector<pair<int,int> >& obj,vector<vector<unsigned char> >& bw,int minradius,int maxradius,int times,int limit)
{
    int rad=maxradius-minradius;

    vector<circle> ans;

    set<pair<int,int> > used;

    for (int t=0; t<times; t++)
    {
        vector<map<pair<int,int>,vector<pair<int,int> > > > acc(rad);

        vector<circle> hough_space;

        for (auto &p:obj)
        {
            int y=p.first,x=p.second;
            if (used.count(p))
                continue;
            double Gy=mask_response(Ly,bw,y,x);
            double Gx=mask_response(Lx,bw,y,x);


            double len=hypot(Gx,Gy);
            Gx/=len,Gy/=len;

            for (int r=0; r<rad; r++)
            {
                for (int sign=1; sign<=1; sign+=2)
                {
                    int yc=y+sign*(r+minradius)*Gy;
                    int xc=x+sign*(r+minradius)*Gx;


                    for (int dy=-1; dy<=1; dy++)
                        for (int dx=-1; dx<=1; dx++)
                            acc[r][ {yc+dy,xc+dx}].push_back(p);
                    //cout << y << ' ' << x << ' ' << r+minradius << ' ' << sign << endl;
                }
            }

        }


        for (int r=0; r<rad; r++)
            for (auto &p:acc[r])
                hough_space.push_back(circle{p.first,r+minradius,t});

        sort(hough_space.begin(),hough_space.end(),[&](auto a,auto b)
        {
            return acc[a.r-minradius][a.coord].size()>
                   acc[b.r-minradius][b.coord].size();
        });

        int a=acc[hough_space[0].r-minradius][hough_space[0].coord].size();
        if (a<limit)
        {
            break;
        }
        else
        {
            ans.push_back(hough_space[0]);
            limit/=2;
            for (auto &p:acc[hough_space[0].r-minradius][hough_space[0].coord])
                used.insert(p);
        }
    }

    return ans;
}

void ball(circle& c,int r,int g,int b,table& T,vector<circle>& v)
{
    int y=c.coord.first-c.r;
    int x=c.coord.second;

    double rad=T.multiplier(y)*real_ball_diameter/2;

    y+=rad;

    c.coord.first=y;
    c.r=rad;
    c.RGB= {r,g,b};


    if (!T.in(y,x-rad))
        return;
    if (!T.in(y,x+rad))
        return;
    if (!T.in(y-rad,x))
        return;

    v.push_back(c);

}

void work(_rgb)
{
    vector<vector<unsigned char> > bw(height,vector<unsigned char>(width));

    for (int i=0; i<height; i++)
    {
        for (int j=0; j<width; j++)
        {
            int r=R[i][j],g=G[i][j],b=B[i][j];
            //bw[i][j]=(r+g+b)/3;
            bw[i][j]=max(r-g/2,0);
        }
    }

    //G=B=R=bw;

    sobel(R,G,B,bw,16);

    auto upper_side=hough(R,1,height/2,        width/3, 2*width/3,   0,height/2,  90,91);
    auto down_side =hough(R,height/2,height-1, width/3, 2*width/3,   0,height/2,  90,91);



    for (auto &s:upper_side)
    {
        int rho=s.second;
        int eps=-4;

        if (rho<=abs(eps))
            continue;

        int cnt=0,gcnt=0,rcnt=0;
        for (int x=width/3; x<2*width/3; x++)
        {
            if (isgreen(Rsrc[rho-eps][x],Gsrc[rho-eps][x],Bsrc[rho-eps][x]))
                ++gcnt;


            if (!isgreen(Rsrc[rho+eps][x],Gsrc[rho+eps][x],Bsrc[rho+eps][x]))
                ++rcnt;

            ++cnt;
        }


        if (gcnt>cnt*0.9&&rcnt>cnt*0.9)
        {
            T.yu=rho;
            break;
        }
    }

    for (auto &s:down_side)
    {
        int rho=s.second;
        int eps=4;

        if (rho<=abs(eps))
            continue;

        int cnt=0,gcnt=0,rcnt=0;
        int dy=height/2;
        for (int x=width/3; x<2*width/3; x++)
        {
            if (isgreen(Rsrc[dy+rho-eps][x],Gsrc[dy+rho-eps][x],Bsrc[dy+rho-eps][x]))
                ++gcnt;

            if (!isgreen(Rsrc[dy+rho+eps][x],Gsrc[dy+rho+eps][x],Bsrc[dy+rho+eps][x]))
                ++rcnt;

            ++cnt;
        }


        if (gcnt>cnt*0.7&&rcnt>cnt*0.7)
        {
            T.yd=dy+rho;
            break;
        }
    }

    if (T.yd==-1||T.yu==-1)
    {
        throw new invalid_argument("can't find the table");
    }


//                                               x0      x1         minmaxdist  minmaxangle
    auto left_side=hough(R,T.yu,T.yd+1,  0,      width/2,   0,width/2,  1,45);
    auto right_side=hough(R,T.yu,T.yd+1, width/2,width,     0,width/2,  -44,0);

    if (left_side.empty()||right_side.empty())
    {
        throw new invalid_argument("can't find the table");
    }

    pair<int,int> L= {-1,-1};

    const static double pi=acos(-1);
    const static int epsx=4;

    int Lwhite=-1,Rwhite=-1;


    for (auto &s:left_side)
    {
        int rho = s.second;
        double theta = (1+s.first)*pi/180;
        int dy=T.yu;
        int dx=0;

        int cnt=0,gcnt=0;

        for (int y=T.yu+1; y<T.yd; y++)
        {
            int x=rho/cos(theta)-(y-dy)*tan(theta)+dx+epsx;
            if (isgreen(Rsrc[y][x],Gsrc[y][x],Bsrc[y][x]))
                ++gcnt;
            int x1=rho/cos(theta)-(y-dy)*tan(theta)+dx-5*epsx;
            if (Rsrc[y][x1]>=128&&Gsrc[y][x1]>=128&&(y-T.yu)<(T.yd-T.yu)*0.75){
                Lwhite=y;
            }
            ++cnt;
        }

        if (gcnt>=cnt*0.7)
        {
            L=s;
            break;
        }
    }

    if (L==pair<int,int> {-1,-1})
    {
        throw new invalid_argument("can't find the table");
    }


    T.xul=X(L,0,1);
    T.xdl=X(L,T.yd-T.yu,1);

    pair<int,int> R_= {-1,-1};
    for (auto &s:right_side)
    {
        int rho = s.second;
        double theta = (-44+s.first)*pi/180;
        int dy=T.yu;
        int dx=width/2;

        int cnt=0,gcnt=0;

        for (int y=T.yu+1; y<T.yd; y++)
        {
            int x=rho/cos(theta)-(y-dy)*tan(theta)+dx-epsx;
            if (isgreen(Rsrc[y][x],Gsrc[y][x],Bsrc[y][x]))
                ++gcnt;

            int x1=rho/cos(theta)-(y-dy)*tan(theta)+dx+5*epsx;
            if (Rsrc[y][x1]>=128&&Gsrc[y][x1]>=128&&(y-T.yu)<(T.yd-T.yu)*0.75){
                Rwhite=y;
            }

            ++cnt;
        }

        if (gcnt>=cnt*0.7)
        {
            R_=s;
            break;
        }
    }

    T.ymid=(Lwhite+Rwhite)/2+2;

    if (R_==pair<int,int> {-1,-1})
    {
        throw new invalid_argument("can't find the table");
    }

    T.xur=width/2+X(R_,0,-44);
    T.xdr=width/2+X(R_,T.yd-T.yu,-44);

    to_src(R,G,B);

    for (int i=0; i<height; i++)
    {
        for (int j=0; j<width; j++)
        {
            int r=R[i][j],g=G[i][j],b=B[i][j];
            bw[i][j]=(r+g+b)/3;
        }
    }

    sobel(R,G,B,bw,48);

    dsu<pair<int,int> > U;

    vector<pair<int,int> > borderpixels;

    double avb=0;
    int sz=0;

    for (auto &p:T.iterate())
    {
        int i=p.first,j=p.second;
        if (isgreen(Rsrc[i][j],Gsrc[i][j],Bsrc[i][j]))
        {
            avb+=Bsrc[i][j];
            ++sz;
        }
        if (R[i][j]&&T.in(i,j))
        {
            auto pr=make_pair(i,j);
            borderpixels.push_back(pr);

            int Gx=mask_response(Lx,bw,i,j);
            int Gy=mask_response(Ly,bw,i,j);
            auto neigh=orthgrad(i,j,Gx,Gy);
            auto n1=neigh.first,n2=neigh.second;

            if (R[unpair(n1)])
            {
                U.union_sets(pr,n1);
            }
            if (R[unpair(n2)])
            {
                U.union_sets(pr,n2);
            }
        }
    }
    avb/=sz;


    /*static mt19937 gen(time(NULL));
    static uniform_int_distribution<int> dis(0,(1<<24)-1);

    map<pair<int,int>,int> objclr;
    for (auto &p:borderpixels)
        if (U[p]==p)
            objclr[p] = dis(gen);



    for (auto &p:borderpixels)
    {
        int clr=objclr[U[p]];
        R[unpair(p)]=clr%256;
        clr/=256;
        G[unpair(p)]=clr%256;
        clr/=256;
        B[unpair(p)]=clr%256;
        clr/=256;
    }*/

    map<pair<int,int>,vector<pair<int,int> > > objects;

    for (auto &p:borderpixels)
        objects[U[p]].push_back(p);

    to_src(R,G,B);

    const int WPYobjectsize=30;
    const int Robjectsize=8;

    double min_ball_size=T.far_side()/real_table_length*real_ball_diameter/2;
    double max_ball_size=T.close_side()/real_table_length*real_ball_diameter/2;



    for (auto &p:objects)
    {
        auto &v=p.second;
        if (v.size()>=WPYobjectsize)
        {
            auto c = hough_circle(v,bw,min_ball_size,max_ball_size+2,2,30);
            for (int i=0; i<(int)(c.size()); i++)
            {
                c[i].RGB= {0,0,0};

                double ang=acos(-1)/24;

                int reps=0;

                int y=c[i].coord.first-(c[i].r-reps)*sin(ang);
                int x1=c[i].coord.second-(c[i].r-reps)*cos(ang);
                int x2=c[i].coord.second+(c[i].r-reps)*cos(ang);

                int yellows=0,whites=0,pinks=0,all=0;

                //cout << endl;
                for (int x=x1; x<=x2; x++)
                {
                    if (isgreen(Rsrc[y][x],Gsrc[y][x],Bsrc[y][x]))
                        continue;
                    ++all;
                    if (isyellow(Rsrc[y][x],Gsrc[y][x],Bsrc[y][x]))
                    {
                        ++yellows;
                    }
                    if (iswhite (Rsrc[y][x],Gsrc[y][x],Bsrc[y][x]))
                    {
                        ++whites ;
                    }
                    if (ispink  (Rsrc[y][x],Gsrc[y][x],Bsrc[y][x]))
                    {
                        ++pinks  ;
                    }
                    //R[y][x]=G[y][x]=B[y][x]=0;
                }


                if (yellows>=0.6*all)
                {
                    c[i].RGB= {255,255,0};
                    circles.push_back(c[i]);
                }
                else if (pinks>=0.5*all)
                {
                    c[i].RGB= {218,179,179};
                    circles.push_back(c[i]);
                }
                else if (whites>=0.6*all)
                {
                    c[i].RGB= {255,255,255};
                    circles.push_back(c[i]);
                }
            }
        }
        if (v.size()>=Robjectsize)
        {
            auto c = hough_circle(v,bw,1,5,1,20);
            if (c.size())
            {
                c[0].RGB= {0,0,0};
                for (int r=7; r>=6; r--)
                {
                    int y=c[0].coord.first,x=c[0].coord.second;
                    int kol=16;
                    double ang1=acos(-1)*5/4;
                    double ang2=acos(-1)*7/4;
                    double step=(ang2-ang1)/kol;

                    set<pair<int,int> > cnt,redcnt,blackcnt,bluecnt,browncnt,greencnt;

                    for (int i=0; i<=kol; i++)
                    {
                        double ang=ang1+step*i;
                        int y1=y-r*sin(ang);
                        int x1=x+r*cos(ang);

                        cnt.insert({y1,x1});
                        if (isbrown(Rsrc[y1][x1],Gsrc[y1][x1],Bsrc[y1][x1]))
                        {
                            browncnt.insert({y1,x1});
                        }
                        if (isblack(Rsrc[y1][x1],Gsrc[y1][x1],Bsrc[y1][x1]))
                        {
                            blackcnt.insert({y1,x1});
                        }
                        if (isgreen_ball(Rsrc[y1][x1],Gsrc[y1][x1],Bsrc[y1][x1],avb))
                        {
                            greencnt.insert({y1,x1});
                        }
                        if (isblue(Rsrc[y1][x1],Gsrc[y1][x1],Bsrc[y1][x1]))
                        {
                            bluecnt.insert({y1,x1});
                        }
                        if (isred(Rsrc[y1][x1],Gsrc[y1][x1],Bsrc[y1][x1]))
                        {
                            redcnt.insert({y1,x1});
                        }
                    }

                    if (blackcnt.size()>=0.6*cnt.size())
                    {
                        ball(c[0],0,0,0,T,circles);
                        break;
                    }
                    else if (browncnt.size()>=0.5*cnt.size())
                    {
                        ball(c[0],128,83,38,T,circles);
                        break;
                    }
                    else if (greencnt.size()>=0.6*cnt.size())
                    {
                        ball(c[0],0,255,63,T,circles);
                        break;
                    }
                    else if (redcnt.size()>=0.6*cnt.size())
                    {
                        ball(c[0],255,0,0,T,circles);
                        break;
                    }
                    else if (bluecnt.size()>=0.6*cnt.size())
                    {
                        ball(c[0],0,0,255,T,circles);
                        break;
                    }
                }
            }
        }
    }
    sort(circles.begin(),circles.end());
    for (int i=0; i<(int)(circles.size()); i++)
    {
        for (int j=i+1; j<(int)(circles.size()); j++)
        {
            if (circles[i].step<circles[j].step)
                if (circles[i].RGB==circles[j].RGB)
                    circles[j].r=0;
        }
    }
}


void getPic(string name,_rgb)
{
    c=SOIL_load_image(name.c_str(),&width,&height,0,SOIL_LOAD_RGBA);
    R=vector<vector<unsigned char> >(height,vector<unsigned char>(width));
    G=vector<vector<unsigned char> >(height,vector<unsigned char>(width));
    B=vector<vector<unsigned char> >(height,vector<unsigned char>(width));

    for (int i=0; i<4*width*height; i++)
    {
        int p=i/4;
        if (i%4==0)
            R[p/width][p%width]=c[i];
        if (i%4==1)
            G[p/width][p%width]=c[i];
        if (i%4==2)
            B[p/width][p%width]=c[i];
    }
}

void work(string s)
{
    vector<vector<unsigned char> > R,G,B;
    getPic(s,R,G,B);
    Rsrc=R,Gsrc=G,Bsrc=B;
    work(R,G,B);
    form(R,G,B,c);
}

int table_window;
int table_width,table_height;

void display_table()
{
    glutSetWindow(table_window);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    glBindTexture(GL_TEXTURE_2D, texid[table_window]);

    glEnable(GL_TEXTURE_2D);

    glBegin(GL_QUADS);
    glColor3d(1,1,1);

    glTexCoord2d(0, 0);
    glVertex2i(0,   0);

    glTexCoord2d(0, 1);
    glVertex2i(0,   table_height);

    glTexCoord2d(1, 1);
    glVertex2i(table_width, table_height);

    glTexCoord2d(1, 0);
    glVertex2i(table_width, 0);
    glEnd();

    double R=real_ball_diameter/(real_table_length+2*real_ball_diameter)*table_width/2;

    for (auto &c:circles)
    {
        if (!c.r)
            continue;

        glBegin(GL_TRIANGLE_FAN);
        glColor3d(c.RGB[0]/255.0,c.RGB[1]/255.0,c.RGB[2]/255.0);
        const int p=60;
        const double pi=acos(-1);

        int r=c.r;
        int y=c.coord.first+r/8;
        int x=c.coord.second;

        double Y=T.realY(y,x,table_width,table_height);
        double X=T.realX(y,x,table_width);

        glVertex2i(X,Y);
        for (int i=0;i<=p;i++){
            double ang=i*2*pi/p;
            glVertex2i(X+R*sin(ang),Y+R*cos(ang));
        }

        glEnd();
    }

    glutSwapBuffers();
}


void draw_table()
{
    unsigned char* c;
    c=SOIL_load_image("img/table.png",&table_width,&table_height,0,SOIL_LOAD_RGBA);

    glutInitWindowPosition(min(1000,3*width/4),50);
    table_window = glutCreateWindow("table drawing");
    glutReshapeWindow(5*table_width/12,5*table_height/12);
    glutDisplayFunc(display_table);

    glGenTextures(1, &texid[table_window]);

    glBindTexture(GL_TEXTURE_2D, texid[window]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, table_width, table_height, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, c);
    initGL(table_width,table_height);



}

int main(int argc, char **argv)
{
    if ( argc < 1)
    {
        return -1;
    }

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE);

    work(string(argv[1]));
    glutInitWindowPosition(0,50);
    window = glutCreateWindow(argv[1]);
    glutDisplayFunc(display);



    glGenTextures(1, &texid[window]);

    glBindTexture(GL_TEXTURE_2D, texid[window]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, c);
    initGL(width,height);

    glutKeyboardFunc(keyboard);

    glutMouseFunc(mouse);
    glutReshapeWindow(3*width/4,3*height/4);

    draw_table();

    glutMainLoop();

    glDeleteTextures(1, &texid[window]);
    glDeleteTextures(1, &texid[table_window]);

    return 0;
}
