
import 'dart:math';
import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter/animation.dart';
import 'package:flutter/material.dart';
import 'package:vector_math/vector_math.dart' hide Colors; // Colors esta definida tanto en libreria material como en Vector

import 'package:sensors_plus/sensors_plus.dart';

import 'package:flutter/services.dart'; // para evitar rotacion entre otros

// SPH fluid
main() {
  
  WidgetsFlutterBinding.ensureInitialized();// para evitar que gire la pantalla agregar libreria services
  SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp])// para evitar que gire la pantalla agregar libreria services
    .then((_) {// para evitar que gire la pantalla agregar libreria services
      runApp(new MaterialApp(home: new DemoPage()));
      });

}

class DemoPage extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMediaQuery(context));
    return new Scaffold(
      body: new DemoBody(
        screenSize: MediaQuery.of(context).size,
      ),
    );
  }
}

class DemoBody extends StatefulWidget {
  final Size screenSize;

  DemoBody({Key key, @required this.screenSize}) : super(key: key);

  @override
  State<StatefulWidget> createState() {
    return new _DemoBodyState();
  }
}


class _DemoBodyState extends State<DemoBody> with TickerProviderStateMixin {
  AnimationController animationController;
  final nodeList = <Node>[];
  
    
  final genericParticles1 = <GenericParticle>[];
  //final genericParticles2 = <GenericParticle>[];


  //   final nodeList = <Node>[]; // Particles
  //final sub_nh = <Node>[]; // Sub Particulas para integrar a lista neighborhoodList dentro de Neighborhood nh
  final nh = <Neighborhood>[]; // Neighbors
  final grid = <GenericParticle>[]; // Particles
  //final grid0 = <GenericParticle>[]; // Particles

  double gyroX = 0;
  double gyroY = 0;
  double gyroZ = 0;

  @override
  void initState() {
    super.initState();

    
      // Para interaccion con girospio, acelerometro, // ESTO SOLO FUNCIONA CUANDO SE SELECCIONA EL EMULADOR pixel 3 XL API 29
      gyroscopeEvents.listen((GyroscopeEvent event) { // agregar libreria sensor_plus.dart
      setState(() {

        gyroX = event.x;
        gyroY = event.y;
        gyroZ = event.z;
      });
    });
    
 

// Variables pcisph
    final numNodes = 10*10; // ********************************************
    final int numMaxVecinos = 2;
    double radioParticula = 1; // 0.03 valor original
    double H = 7.5 * radioParticula; // 0.18
    int solverSteps = 1;
    int fps = 13;
    double spacing = radioParticula + 1 ; // al modificar este valor se evita el 0
    //double spacing = radioParticula + 1  ; // al modificar este valor se evita el 0
    
     //double viewWidth = 20.0; // original 20
    double viewWidthDouble = widget.screenSize.width; // original 20
    int viewWidthInteger =viewWidthDouble.floor();
    double viewWidth = viewWidthInteger.toDouble();   
    
    double particleSize = 7 * radioParticula * widget.screenSize.width / viewWidth;
    
    
    //double HSQ = H * H; // radius^2 for optimization
    double sizeCelda = H;
    

    print(viewWidth); // 411

    
    double viewHeightDouble = (widget.screenSize.height);
    int viewHeightInt = viewHeightDouble.floor();
    double viewHeight =viewHeightInt.toDouble();
    print(viewHeight); //797


    int anchoCelda = (viewWidth / sizeCelda).ceil();
    print(anchoCelda);
    int altoCelda = (viewHeight / sizeCelda).ceil();
    print(altoCelda);
    int numCeldas = anchoCelda * altoCelda;
    print("numCeldas");
    print(numCeldas);
    print("sizeCelda");
    print(sizeCelda);
    print("particle size");
    print(0.5 * radioParticula * widget.screenSize.width / viewWidth); // tamaño de la particula)

    //double POLY6 = 315.0 / (65.0 * 3.1416 * pow(H, 9.0)); // El núcleo poly6 también se conoce como núcleo polinomial de sexto grado. 
   

    int maxParticles = numNodes;
    var gridIndices = List<Vector2>.filled(maxParticles, Vector2(0.0,0.0)); // lista con 25 elementos tipo vector inicializados en 0
    var xlast = List<Offset>.filled(numNodes, Offset(0.0,0.0)); // lista con 5 elementos tipo offset inicializado en 0
    //double eps = 0.0000001;
    double eps = 1; // Limite inferior del radio H para evitar 0
    double eps2 = eps * eps;
    double kernel = 20.0 / (2.0 * 5.0 * H * H);
    double kernelNorm = 30.0 / (2.0 * 5.0 * H * H);
    double rigidez = 0.08;
    double densidad0 = 45.0;
    double aproximacionRigidez = 0.1;
    
    
    
    double dt = ((1.0 / fps) / solverSteps);
    Offset g = Offset(0.0, 9.81); // -9.81 valor original
    double dt2 = dt * dt;
    double tensionSuperficial = 0.000001;
    double viscosidadLineal = 0.05;
    double viscosidadCuad = 0.05;
    var xprojected = List<Vector2>.filled(maxParticles, Vector2(0.0,0.0)); // lista con elementos tipo vector2 inicializados en 0
    //var boundaries = List<Vector3>.filled(maxParticles, Vector3(0.0,0.0,0.0)); // lista con 25 elementos tipo vector3 inicializados en 0
    List<Vector3> boundaries = [];
    boundaries.add(Vector3(1, 0, 0));
    boundaries.add(Vector3(0, 1, 0));
    boundaries.add(Vector3(-1, 0, -viewWidth));
    boundaries.add(Vector3(0, -1, -viewHeight));

//COMENTAR ESTA PARTE ORIGINAL
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///

    var listaxy = [];
    int num = sqrt(numNodes).ceil();

    
    double x = 0.3 * viewWidth;
    //double y = 0.5 * viewHeight;
    //print(x0);
    
    for (double i = 0, y = 0.5 * viewHeight; i < num; i++, y -= 2 * radioParticula + spacing ) // recorre todas las particulas para acomodarlas en rompimiento de presa.
    {
        Random random = new Random();
        for (int j = 0; j < num; j++) // aqui se actuliza valor y, dentro no lo reconoce
        {

            listaxy.add([x,y]);
  
            
            x += 2.0 * radioParticula + spacing + 0.1*random.nextDouble() ; // actualiza valor de primera posicion del vector start sumando esta operacion 
 
        }
        x = 0.3 * viewWidth; // resetea posicion x
 

    }
    
    //print(listaxy);

    
    // INICIALIZACION PARTICULAS principales---------------------------------------------------------------------------------------------------------

    Iterable.generate(numNodes).forEach((i) { // inicializacion de particulas
      
      double valx = (listaxy[i][0]); // variable para posicion inicial en x
      double valy = (listaxy[i][1]); // variable para posicion inicial en y

       nodeList.add( // inicializacion de valores no inicializados en la calse Node
        new Node(
          id: i, 
          screenSize: widget.screenSize, 
          //size: 6 * radioParticula * widget.screenSize.width / viewWidth, // tamaño de la particula
          size: particleSize, // tamaño de la particula
          position: Offset(valx, valy),
          velocidad: Offset(0.0, 0.0),
          
          )
        );
      } 
    );
// INICIALIZACION PARTICULAS fin ---------------------------------------------------------------------------------------------------------

// INICIALIZACION PATICULAS GENERICAS PARA ESPEJO DE PARTICULAS PRINCIPALES

 Iterable.generate(numNodes).forEach((i) { // inicializacion de particulas  

      double valx = (listaxy[i][0]); // variable para posicion inicial en x
      double valy = (listaxy[i][1]); // variable para posicion inicial en y

      
      genericParticles1.add( // inicializacion de valores no inicializados en la calse Node
        new GenericParticle
          ( position: Offset(valx, valy),
            velocidad: Offset(0.0, 0.0),
            //n1:GenericParticle( position: Offset(0.0, 0.0), velocidad: Offset(0.0, 0.0)), // opcional
            )
        );
      } 
    );




// INICIALIZACION vecinos numNeighbors---------------------------------------------------------------------------------------------------------

    Iterable.generate(numNodes).forEach((i) { // inicializacion de particulas
      
      nh.add( // inicializacion de valores no inicializados en la calse Node
        new Neighborhood(
                    position: Offset(0.0, 0.0),
                    velocidad: Offset(0.0, 0.0),
                    r: List<double>.filled(numNodes, 0.0),
                    neighborhoodList: List<GenericParticle>.filled(numNodes, // 64 vecinos tipo ParticlesNeighborhood // neighborhoodList es una lista a la que se le asigna otra lista de varios elementos ya inicializados          
                            GenericParticle( 
                                            position: Offset(0.0, 0.0),
                                            velocidad: Offset(0.0, 0.0),
                                            )
                                                        )
                         ), 

        );
      } 
    );

void move(){


  for (int i = 0; i < solverSteps; i++) 
  {
          grid.clear();
            
           Iterable.generate(numCeldas).forEach((k) { // inicializacion de particulas  
                    grid.add( // inicializacion de valores no inicializados en la calse Node
                      new GenericParticle
                        ( position: Offset(0.0, 0.0), 
                          velocidad: Offset(0.0, 0.0),
                          //n1:GenericParticle( position: Offset(0.0, 0.0), velocidad: Offset(0.0, 0.0)),
                          )
                      );
                    } 
                  );

          for (int j=0 ,k = maxParticles; j < genericParticles1.length; j++,k++){
            
            // ApplyExternalForce
            //genericParticles1[j].velocidad += g* dt ; // gravedad normal
            //genericParticles1[j].velocidad += g* dt *gyroY; // gravedad modificada 
            genericParticles1[j].velocidad += g* dt *(gyroY+1); // gravedad modificada temporalmente
            
            // Integrate
            xlast[j] = genericParticles1[j].position; //guarda posicion de particula actual(i)
            genericParticles1[j].position += genericParticles1[j].velocidad * dt ; //  suma y actualiza posicion debida a la velocidad y tiempo 

            // GridInsert
            int indice = k-maxParticles;  
            double px = (genericParticles1[j].position.dx / sizeCelda);//escala posicion al grid dividiendo entre cell size para quedar en un rango entre 0 y 111 en coordenada X de la coleccion grid
            int xind = px.ceil();
            double py = (genericParticles1[j].position.dy / sizeCelda);
            int yind = py.ceil();
            xind = max(1,(min(anchoCelda - 2, xind))); // si xind esta al borde toma la posicion extrema a lo ancho y le resta 2 posiciones para poder evaluar los vecinos de todos los lados
            yind = max(1,(min(altoCelda - 2, yind)));
            genericParticles1[j].n1 = grid[xind+yind*anchoCelda].n1; // guardar genericparticle grid en variable n tipo genericparticle dentro de nodelist
            grid[xind+yind*anchoCelda].n1 = genericParticles1[j];
            gridIndices[indice] = Vector2(xind.toDouble(), yind.toDouble()); //va

          }

//------------------------------// PressureStep
          //print("--------------INICIO ciclo PRESSURESTEP--------------");
          for (int j = 0; j < genericParticles1.length; j++)
          {
            //print("--------------INICIO CICLO INTERNO FOR PRESURESTEP--------------");

            Vector2 ind = Vector2(gridIndices[j][0],(gridIndices[j][1]) * anchoCelda);

            nh[j].numNeighbors = 0;
            
            double dens = 0.0;
            double dens_proj = 0.0;

            
            for (int ii = ind[0].ceil() - 1; ii <= ind[0] + 1; ii++) //
            {
              //print("[ii]****");
               // print(ii);
                
                for (int jj = ind[1].ceil() - anchoCelda; jj <= ind[1] + anchoCelda; jj += anchoCelda)
                { 
                    for (GenericParticle pgrid = grid[ii+jj].n1; pgrid != null; pgrid= pgrid.n1) // asigna a particula nueva pgrid el valor de particula grid que antes guardo valores de las particulas generales == static vector<Particle*> grid(numCeldas); // static const int numCeldas = anchoCelda * altoCelda;
                    { 
                        Offset dx_offset = pgrid.position - genericParticles1[j].position;
                        Vector2 dx = Vector2(dx_offset.dx, dx_offset.dy); // convertir offset a vector
                    
                        
                        double r2 = dx[0]*dx[0] + dx[1]*dx[1] ; // suma del cuadrado de todas las entradas de la matriz.
                      
                        
                        // si r2 < eps2 = true O si r2 > H * H continua la proxima
                        if (r2 < eps2 || r2 > H * H){continue;} // termina iteracion for actual y va con la siguiente
                        
                        double r = sqrt(r2);
                        
                        double a = 1.0 - r / H; // nucleo spike
                        
                        dens += pgrid.masa * a * a * a * kernel;
                    
                        dens_proj += pgrid.masa * a * a * a * a * kernelNorm;
                    

                        
                        if (nh[j].numNeighbors < numMaxVecinos) // 0 < 64
                        {
                            //cout << nh[i].numNeighbors << endl; //-- 0,1,2,3,4,5,6,7,8,9,10,11 dentro del ciclo for hasta aproximadamente 11
                            nh[j].neighborhoodList[nh[j].numNeighbors] = pgrid; // el valor de &pj se asigna nh

                            
                            nh[j].r[nh[j].numNeighbors] = r; // double r = sqrt(r2);
                            
                            ++nh[j].numNeighbors; // aumenta en una unidad hasta aproximadamente 11 dentro de este ciclo
                     
                            
                        }
                    }
                }
            }

            //print("inicio presustep interno");

            genericParticles1[j].densidad = dens;
           // double densi0 = genericParticles1[j].densidad;
           // print("pi.d");
           // print(densi0.toStringAsFixed(5));

            genericParticles1[j].densidad_v = dens_proj;
            //double densi1 = genericParticles1[j].densidad_v;
            //print("pi.dv");
            //print(densi1.toStringAsFixed(5));
       
            
            genericParticles1[j].presion = rigidez * (dens - genericParticles1[j].masa * densidad0);
            //double presi0 = genericParticles1[j].presion;
            //print("pi.p");
           // print(presi0.toStringAsFixed(5));
  
            
            genericParticles1[j].presion_v = aproximacionRigidez * dens_proj;
            //nodeList[j].notePaint.color = Color.fromARGB(255, 255, ((nodeList[i].presion_v)*10000).toInt(), 255); // color dinamico segun fuerza
//            print(((genericParticles1[j].presion_v)*100000).toInt());
            //double presi1 = genericParticles1[j].presion_v;
            //print("pi.pv");
            //print(presi1.toStringAsFixed(5));
        
            //print("fin presustep interno");
            
        

          }
         

          //print("--------------INICIO ciclo PROJECT--------------");
          // Project --------------------------------------------------
          for (int j = 0; j < genericParticles1.length; j++) ///////////////////aQUI--------------------
          {
           // print("--------------INICIO ciclo PROJECT INTERNO--------------");
              Vector2 xx = Vector2(genericParticles1[j].position.dx, genericParticles1[j].position.dy); // guarda posicion de particula evaluada
    

              // static vector<Neighborhood> nh(numParticles);
              for (int k = 0; k < nh[j].numNeighbors; k++) // para cada uno de los vecinos de la particula de la coleccion
              {
                  //GenericParticle pj = nh[i].neighborhoodList[j]; //la variable pj de tipo particula tomara el valor del vecino j de la coleecion de vecino de i
                  double r = nh[j].r[k]; // variable vector r almacena la posicion correspoindiente a j en el rango declarado dentro de la estructura Neighborhood correspondiente a nh r(numMaxVecinos)

                  
                  Offset dx_0 = nh[j].neighborhoodList[k].position - genericParticles1[j].position; // distancia entre particula i y particula vecina j // offset
                  Vector2 dx = new Vector2(dx_0.dx, dx_0.dy); // distancia entre particula i y particula vecina j // vector
     

                  double a = 1.0 - r / H;
        
                 
                  double d = dt2 * ((genericParticles1[j].presion_v + nh[j].neighborhoodList[j].presion_v) * a * a * a * kernelNorm + (genericParticles1[j].presion + nh[j].neighborhoodList[k].presion) * a * a * kernel) / 2.0;
    
                  xx -= (dx * d) / (genericParticles1[j].masa * r );


                  // surface tension applies if the particles are of the same material
                  // this would allow for extensibility of multi-phase     
                  // if (genericParticles1[j].masa == nh[j].neighborhoodList[k].masa){
                  //     xx += dx * ((tensionSuperficial / genericParticles1[j].masa) * nh[j].neighborhoodList[k].masa * a * a * kernel) ;
         
                  //   }
             
                  Offset dv_offset = genericParticles1[j].velocidad - nh[j].neighborhoodList[k].velocidad;
                  
                  Vector2 dv = new Vector2(dv_offset.dx, dv_offset.dy);
    
                  double u = dv.dot(dx); // producto escalar de vector2d dv con dx
      


                  if (u > 0)
                  {
                      u /= r;
           
                      double a = 1 - r / H;
           
                      double I = 0.5 * dt * a * (viscosidadLineal * u + viscosidadCuad * u * u);
           
                      xx -= dx * (dt * I);
           
                  }
                  //print("--------------FIN ciclo PROJECT INTERNO--------------");
              }
              //static vector<Vector2d> xprojected(numParticles); // variable global
              xprojected[j] = xx; // relaxation
      
          }
         // print("--------------FIN ciclo PROJECT--------------"); // REVISADO
          
         
         // print("--------------INICIO ciclo CORRECT--------------");
          for (int j = 0; j < genericParticles1.length; j++) // para cada particula 
              {
               // print("--------------INICIO ciclo CORRECT INTERNO--------------");
                //auto& p = particles[i];// dada una particula de la coleccion matriz particulas asignar variable p
                Offset relax = Offset(xprojected[j][0],xprojected[j][1]);
      

                genericParticles1[j].position = relax; // posicion de la particula igual a xprojected (relaxation) calculado en funcion project
                genericParticles1[j].velocidad = (genericParticles1[j].position - xlast[j]) / dt; // velocidad igual a la posicion menos posicion anterior dividido entre el tiempo
              
                                
               // print("--------------FIN ciclo CORRECT INTERNO--------------");
              }
 


         // print("--------------INICIO ciclo ENFORCEBOUNDARY--------------");

          for (int j = 0; j < genericParticles1.length; j++)
          { // para cada particula
              for (int k = 0; k < boundaries.length; k++) // static vector<Vector3d> boundaries = vector<Vector3d>(); // variable global
              {//var boundaries = List<Vector3>.filled(maxParticles, Vector3(0.0,0.0,0.0)); // lista con 25 elementos tipo vector3 inicializados en 0

               // print("--------------INICIO ciclo interno boundaries--------------");
                // double d = (genericParticles1[j].position.dx * boundaries[k][0] + genericParticles1[j].position.dy * boundaries[k][1] - boundaries[k][2]); // original
                double d = (genericParticles1[j].position.dx * boundaries[k][0] + genericParticles1[j].position.dy * boundaries[k][1] - boundaries[k][2]);

  
                  if ((d = max(0.0, d)) < radioParticula)
                    {
                      Offset boundarieAux = Offset(boundaries[k][0] / dt, boundaries[k][1] / dt);
                      //genericParticles1[j].velocidad += (boundarieAux *(radioParticula - d)); // orignal
                      genericParticles1[j].velocidad += (boundarieAux *(radioParticula - d))*1;

                    }
                    //print("--------------fin ciclo interno boundaries--------------");
              }
              
        }
        //print("--------------FIN ciclo ENFORCEBOUNDARY--------------");
        
        //print("--------------FIN -");



      }

  //print("--------------REnder-------------------------");
  for (int i = 0; i < nodeList.length; i++) 
  {
    // ACTUALIZACION POSICION
    nodeList[i].position = genericParticles1[i].position;
    nodeList[i].notePaint.color = Color.fromARGB(255, 0, ((genericParticles1[i].presion_v)*100000).toInt(), 250); // color dinamico segun fuerza
    //print(((genericParticles1[i].presion_v)*100000).toInt());
    //print(jjd.toStringAsFixed(10));
    
  }


}

//*******************************************INICIO CICLO PRINCIPAL******************************************************* */
    animationController = new AnimationController(
        vsync: this, duration: new Duration(seconds: 10))
      ..addListener(() {

       // print("-----------------------primer iteracion----------------------------");
         
  move();


    }
      
    )
      ..repeat();
  }




  @override
  Widget build(BuildContext context) {
    return new Container(
      child: new AnimatedBuilder(
        animation: animationController,
        builder: (context, child) => new CustomPaint(
              size: widget.screenSize,
              painter: new _DemoPainter(widget.screenSize, nodeList),
            ),
      ),
    );
  }
}


class _DemoPainter extends CustomPainter {
  final List<Node> nodeList;
  final Size screenSize;
  var counter = 0;

  _DemoPainter(this.screenSize, this.nodeList);

  @override
  void paint(Canvas canvas, Size size) {
    for (var node in nodeList) {
      node.display(canvas);
    }
  }

  @override
  bool shouldRepaint(_DemoPainter oldDelegate) => true;
}


class Node {
      int id;
      Size screenSize;
      double size;
      Random random;
      Paint notePaint, linePaint;
      
      
      // position, velocity, and mass
      Offset position; // x
      Offset velocidad; // v
      double masa; // m
      // pressure, density, and their variations
      double densidad;
      double densidad_v;
      double presion;
      double presion_v;
      Offset viscosidad;
    
      
      int numNeighbors;
      bool nulidad;
      //GenericParticle n;

  Node(
      {
      @required this.id,
      @required this.screenSize,
      @required this.size,//Tamaño del punto 
      @required this.position,
      @required this.velocidad,
      //@required this.n,
      
      
      this.masa = 1.0,
      this.presion = 0.0,
      this.presion_v = 0.0,
      this.densidad = 0.0,
      this.densidad_v = 0.0,
      
      this.nulidad = true,
      
      }) {
   
    random = new Random(); 
   

    notePaint = new Paint()
      ..color = Colors.blue;

    
    linePaint = new Paint()
      ..color = Colors.blueAccent
      ..strokeWidth = 0.5
      ..style = PaintingStyle.stroke;
  }

 
   void display(Canvas canvas) {
     canvas.drawCircle(position, size, notePaint);
    }

  bool operator ==(o) => o is Node && o.id == id;
  int get hashCode => id;
}


class Neighborhood {
  
  
  
        // position, velocity, and mass
        Offset position;      // x
        Offset velocidad;     // v
        double masa;          // m
        // pressure, density, and their variations
        double densidad;      // d
        double densidad_v;    // dv
        double presion;       // p
        double presion_v;     // pv

        List<GenericParticle>neighborhoodList; // Particles      
        int numNeighbors;
        List<double> r;

  

 Neighborhood(
      {
      
      
        @required this.neighborhoodList ,
        @required this.position,
        @required this.velocidad,
        @required this.r,
                
        this.masa = 1.0,
        this.presion = 0.0,
        this.presion_v = 0.0,
        this.densidad = 0.0,
        this.densidad_v = 0.0,
        

       
        this.numNeighbors = 0,
        

      }) {  }

}

class GenericParticle { // grid
  
        // position, velocity, and mass
        Offset position;      // x
        Offset velocidad;     // v
        double masa;          // m
        // pressure, density, and their variations
        double densidad;      // d
        double densidad_v;    // dv
        double presion;       // p
        double presion_v;     // pv
              
        int numNeighbors;
        bool nulidad;

        GenericParticle n1;
        

        
      GenericParticle( //
        {

        @required this.position,
        @required this.velocidad,
        this.n1,
       
                        
        this.masa = 1.0,
        this.presion = 0.0,
        this.presion_v = 0.0,
        this.densidad = 0.0,
        this.densidad_v = 0.0,
        this.numNeighbors = 0,
        this.nulidad = false
        
        }) 
          { } // para inicializar variables o modificarlas
          //void move(){} // funciones de la clase ejemplo: move
    }
