#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <SFML/Graphics.hpp>
sf::RenderWindow window(sf::VideoMode(1920, 1080), "Sim",sf::Style::Default);
int winx = window.getSize().x;
int winy = window.getSize().y;
float size_ratio = 80; //physics numbers are in SI, computer thinks in pixels
float grav_acc = 9.81;
float grav_constant = 6.67;
float dt = 0.01666667; //framerate and timestep
const int max_edges = 7; //maximum number of edges per vertex, no need to be above number of vertices-1
class vertex;
class edge;
class obstacle;
std::vector<obstacle*> obs;
std::vector<vertex> verticesvec;
std::vector<edge> edgesvec;

float dot(float x1, float y1, float x2, float y2) //dot product
{
	return x1*x2+y1*y2;
}

std::tuple<float, float> projection(float x1, float y1, float x2, float y2, float x3, float y3)//the last two coordinates are for the point 
{
	//orthogonal projection vector
	float proj_scalar = dot(x3-x1, y3-y1, x2-x1, y2-y1)/dot(x2-x1, y2-y1, x2-x1, y2-y1);
	std::tuple<float, float> proj = {x1+(x2-x1)*proj_scalar, y1+(y2-y1)*proj_scalar};
	return proj;
}

class obstacle
{
	public:
		sf:: ConvexShape sprite;
		sf:: ConvexShape box_sprite;
		float angle;
		float points=0;
		float box_factor = 0.4;
		int c[6]; //x1, y1, |x2, y2, x3, y3|<-this must corespond to the hypothenuse
		int x, y, adj, opp; //adjacent and opposite sides
		int box[4]; //each triangle is surrounded by a "box" with dimensions larger than itself so that surrounding objects know when to measure distance
		int innerbox[4]; //(xmin, xmax, ymin, ymax) format, like the one above,  this is a smaller box with dimensions similar to the triangle's
		obstacle(int xpos, int ypos, int a, int o)
		{
			x = xpos; y = ypos;
			adj = a; opp = o;
			c[0] = x; c[1] = y+opp; c[2] = x+adj; c[3] = y+opp; c[4] = x; c[5] = y;
			angle = std::atan2(opp, adj);
			//one or both sides of the triangle may be negative, which makes detrmining wether or not a vertex is in the box very hard
			innerbox[0] = std::min(c[0], c[2]);
                        innerbox[1] = std::max(c[0], c[2]);
                        innerbox[2] = std::min(c[1], c[5]);
                        innerbox[3] = std::max(c[1], c[5]);
			box[0] = innerbox[0]-std::abs(adj*box_factor);
			box[1] = innerbox[1]+std::abs(adj*box_factor);
			box[2] = innerbox[2]-std::abs(opp*box_factor);
			box[3] = innerbox[3]+std::abs(opp*box_factor);
			obs.push_back(this); //add own adress to the list of obstacles
			//the sprite isnt movable
			sprite.setPointCount(3);
			sprite.setPoint(0, sf::Vector2f(c[0], c[1]));
			sprite.setPoint(1, sf::Vector2f(c[2], c[3]));
			sprite.setPoint(2, sf::Vector2f(c[4], c[5]));
			sprite.setFillColor(sf::Color(200, 200, 200));
		}
		void display()
		{
			window.draw(sprite);
			//display_box();
		}
		void display_box()
		{
			box_sprite.setPointCount(4);
			box_sprite.setPoint(0, sf::Vector2f(box[0], box[3]));
			box_sprite.setPoint(1, sf::Vector2f(box[1], box[3]));
			box_sprite.setPoint(2, sf::Vector2f(box[1], box[2]));
			box_sprite.setPoint(3, sf::Vector2f(box[0], box[2]));
			box_sprite.setFillColor(sf::Color(255, 0, 0, 100));
			window.draw(box_sprite);
		}
};

class vertex
{
	public:
		int number;
		float x, y;
		float xspeed, yspeed;
		float xacc, yacc; //acceleration vector
		float xforce, yforce; //force applied on the vertex
		float mass;
		int radius = 10;
		int diam = 2*radius;
		bool yblock = false, xblock = false; //blocks the vertex on the x or y axis, useful for tests
		bool contact = false; //collisions are perfectly inelastic, so the speed is put to 0 when one happens, this allows to only do it at the instant of the collision and not all throughout contact with an obstacle
		sf::CircleShape sprite;
		sf::RectangleShape force_sprite;
		vertex(float xpos, float ypos, int num)
		{
			number = num;
			x = xpos; y = ypos;
			xspeed = 0; yspeed = 0;
			xacc = 0; yacc = grav_acc;
			xforce = 0; yforce = 0; //we dont take the earth's pull into account (acceleration already defined)
			mass = 1;
			//visual aspect
			sprite.setRadius(radius);
			sprite.setFillColor(sf::Color(255, 255, 255));
			sprite.setPosition(x, y);
		}
		void bounds()
		{
			if((y+diam >= winy and yspeed>0) or (y <= 0 and yspeed<0))
			{
				yspeed = 0;
				yforce = 0;
			}
			if((x+diam >= winx and xspeed>0) or (x <= 0 and xspeed<0))
			{
				xspeed = 0;
				xforce = 0;
			}
		}
		void obstacles()
		{
			for(size_t i = 0; i < obs.size(); i++)
			{
				obstacle* o = obs[i];
				//std::cout << "x: " << x << " y: " << y << " box: " << o->box[0] << " ; " << o->box[1] << "; "<< o->box[2] << "; "<< o->box[3] << "\n";
				if(x+radius > o->box[0] and x+radius < o->box[1] and y+radius > o->box[2] and y+radius < o->box[3])//of the center is in that box
				{
					if(x+radius >= o->innerbox[0] and x+radius <= o->innerbox[1])
					{
						//find shortest distance to the surface with orthogonal projection
						std::tuple<float, float> p = projection(o->c[2], o->c[3], o->c[4], o->c[5], x+radius, y+radius);
						float l = std::hypot(std::get<0>(p)-(x+radius), std::get<1>(p)-(y+radius));
						if(l<=radius)
						{
							//eventually adjust position to avoid clipping
							//sprite.setFillColor(sf::Color(255, 0, 0));
							if(contact == false)
							{
								xspeed=0; yspeed=0;//perfectly inelastic collision
								contact = true;
							}
							//simulate normal force
							//find a temporary vector perpendicular to the surface
							float tempy = static_cast<float> (o->c[2]-o->c[4]) / static_cast<float> (o->c[5]-o->c[3]);
							//project the force vector onto it
							std::tuple<float, float> p2 = projection(0, 0, 1, tempy, xforce, yforce);
							//calculate normal reaction by getting length of projected vector
							float f = std::hypot(std::get<0>(p2), std::get<1>(p2));
							//std::cout << "edge vector: " <<  o->c[4]-o->c[2] << ";" << o->c[5]-o->c[3]
							//<< "\nangle: " << o->angle
							//<< "\nnormal vector: 1;" << tempy
							//<< "\nforce vector: " << xforce << ";" << yforce
							//<< "\nprojected vector: " << std::get<0>(p2) << ";" << std::get<1>(p2) 
							//<< "\nforce: " << f << "\n\n";
							float ynorm = std::abs(f*std::cos(o->angle));
                                                        float xnorm = std::abs(f*std::sin(o->angle));
							if(o->opp>0)//slide down
							{
								yforce -= ynorm;
							}
							else
							{
								yforce += ynorm;
							}
							if(o->adj>0)//slide right
							{
								xforce += xnorm;
							}
							else
							{
								xforce -= xnorm;
							}
						}
						else
						{
							//sprite.setFillColor(sf::Color(255, 255, 255));
							contact = false;
						}
					}
					//eventually add case if it hits from under or the side
				}
			}
		}
		void update()
		{
			yforce += mass*grav_acc;
			obstacles();
			xacc = xforce/mass; yacc = yforce/mass;
			xspeed += xacc*dt; yspeed += yacc*dt;
			bounds();
			if(xblock == false)
			{
				x += xspeed*dt*size_ratio;
			}
			if(yblock == false)
			{
				y += yspeed*dt*size_ratio;
			}
			display();
			//reset force values
			xforce = 0, yforce = 0;
			xblock = false; yblock = false;
		}
		void display()
		{
			//display_force();
			sprite.setPosition(x, y);
			window.draw(sprite);
		}
		void display_force() //displays force vector
		{
			float angle;
			float force = std::hypot(xforce, yforce);
			if(xforce != 0)
			{
				angle = std::atan2(yforce, xforce);
			}
			else if(force != 0)
			{
				angle = std::asin(yforce/force);
			}
			force_sprite.setSize(sf::Vector2f(0.01*force*size_ratio, 6));
			force_sprite.setFillColor(sf::Color(0, 0, 255));
			force_sprite.setPosition(x+radius, y+radius);
			float angle_diff = (angle*180/M_PI) - force_sprite.getRotation();
			force_sprite.rotate(angle_diff);
			window.draw(force_sprite);
			std::cout << "number:"  << number << "| x:" << x << "| y:" << y 
			<< "| xforce:" << xforce << "| yforce:" << yforce
			<< "| xspeed:" << xspeed << "| yspeed:" << yspeed
			<< "| xacc:" << xacc << "; yacc:" << yacc << "| angle:" 
			<< angle*180/M_PI << "\n";
		}
};

class edge
{
	public:
		vertex *vertex1, *vertex2;
		float l0; //resting lenght of the spring
		float maxcomp, maxext;
		float damp = 1;
		float k = 5; //spring constant, forces will be calculated using hooke's law
		float force, fx, fy;
		float x1, y1, x2, y2;
		float adj, opp, hyp, angle; //sides of the triangle and orientation
		int thickness = 3;
		int vertex_rad;
		sf::RectangleShape sprite;
		edge(vertex *v1, vertex *v2, float l=300)
		{
			l0 = l;
			maxcomp = l0*0.5;
			maxext = l0*1.25;
			vertex1 = v1; vertex2 = v2;
			vertex_rad = vertex1->radius;
			update();
		}
		float spring()
		{
			return -k*(hyp-l0);
		}
		void update()
		{
			x1 = vertex1->x; y1 = vertex1->y;
			x2 = vertex2->x; y2 = vertex2->y;
			adj = x2 - x1;
			opp = y2 - y1;
			hyp = std::hypot(adj, opp); 
			if(adj != 0)
			{
				angle = std::atan2(opp, adj);
			}
			else if(hyp != 0)
			{
				angle = std::asin(opp/hyp);
			}
			force = spring();
			//hooke's law only applies for "reasonable" extension values of the spring
			if(hyp<maxcomp or hyp>maxext)
			{
				force *=2;
			}
			fx = force*std::cos(angle);
			fy = force*std::sin(angle);
			//apply that force to both ends
			vertex1->xforce -=fx; vertex2->xforce += fx;
			//now y coordinates
			vertex1->yforce -=fy; vertex2->yforce += fy;
			//damping
			if(vertex1->xforce>0){vertex1->xforce -= vertex1->xspeed*damp;}
			if(vertex1->yforce>0){vertex1->yforce -= vertex1->yspeed*damp;}
			if(vertex2->xforce>0){vertex2->xforce -= vertex2->xspeed*damp;}
			if(vertex2->yforce>0){vertex2->yforce -= vertex2->yspeed*damp;}
			display();
		}
		void display()
		{
			sprite.setSize(sf::Vector2f(hyp, thickness));
			sprite.setFillColor(sf::Color(255, 165, 0));
			sprite.setPosition(x1+vertex_rad, y1+vertex_rad);
			float angle_diff = (angle*180/M_PI) - sprite.getRotation();
			sprite.rotate(angle_diff);
			window.draw(sprite);
		}
};

void mesh(int x, int y, int w, int h, int l) //creates a mesh structure
{
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			vertex v(x+j*l, y+i*l, (i*w)+j);
			verticesvec.push_back(v);
		}
	}
	//now build edges
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			int a = (i*w)+j; //position in the array
			if(j<w-1)
			{
				edge e1(&verticesvec[a], &verticesvec[a+1], l);
				edgesvec.push_back(e1);
				if(i>0)
				{
					edge e3(&verticesvec[a], &verticesvec[a-w+1], std::hypot(l,l));
					edgesvec.push_back(e3);
				}
				if(i<h-1)
				{
					edge e4(&verticesvec[a], &verticesvec[a+w+1], std::hypot(l,l));
					edgesvec.push_back(e4);
				}
			}
			if(i<h-1)
			{
				edge e2(&verticesvec[a], &verticesvec[a+w], l);
				edgesvec.push_back(e2);
			}
		}
	}
}

int main()
{
	int sleep;
	mesh(500, 100, 5, 3, 80);
	obstacle o1(400, winy-500, 800, 400);
//-------------------------------------------------//
	while (window.isOpen())
	{
		auto start = std::chrono::steady_clock::now();
		//do stuff here                                             
		window.display();
		window.clear();
		o1.display();
		for(size_t i = 0; i < verticesvec.size(); i++)
		{
			verticesvec[i].update();
		}
		for(size_t i = 0; i < edgesvec.size(); i++)
		{
			edgesvec[i].update();
		}
		//-------------
		auto end = std::chrono::steady_clock::now();
		std::chrono::duration<float> elapsed_seconds = end-start;
		float remainder = dt - elapsed_seconds.count();
		if(remainder > 0)
		{
			sleep = static_cast<int>(remainder*1000000);
			std::this_thread::sleep_for(std::chrono::microseconds(sleep)); //wait for the timestep to be over
		}
		//std::chrono::duration<float> fpscount = std::chrono::steady_clock::now()-start; std::cout << "\nfps: " << 1/fpscount.count() << "\n";
		sf::Event event;
		while (window.pollEvent(event))
		{
		    	if (event.type == sf::Event::Closed)
			{
				window.close();
				return 0;
			}
		}
	}
	return 0;
}
