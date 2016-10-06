/*
---------------------------------------------------------------------------------------
CS599 Project 3
R Mitchell Ralston (rmr5)
10/6/16
--------------------------
Implement Project3 as per assignment spec on BBLearn, adding illumination to the raycaster

Organization:
------------
variables are named with underscore naming convention
functions are named with camelCase naming convention

Issues:
-------

Questions:
---------
---------------------------------------------------------------------------------------
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
 
#define DEFAULT_COLOR 0

// typdefs
typedef struct RGBPixel {
  unsigned char r;
  unsigned char g;
  unsigned char b;
} RGBPixel ;

typedef struct RGBAPixel {
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned char a;
} RGBAPixel ;

typedef struct PPM_file_struct {
  char magic_number;
  int width;
  int height;
  int alpha;
  int depth;
  char *tupltype;
  FILE* fh_out;
} PPM_file_struct ;

typedef struct A_J {
  double A;
  double B;
  double C;
  double D;
  double E;
  double F;
  double G;
  double H;
  double I;
  double J;
} A_J ;

typedef struct has_values {
  int has_width;
  int has_height;
  int has_color;
  int has_position;
  int has_normal;
  int has_radius;
  int has_A;
  int has_B;
  int has_C;
  int has_D;
  int has_E;
  int has_F;
  int has_G;
  int has_H;
  int has_I;
  int has_J;
} has_values ;
  
typedef struct JSON_object {
  char *type;       // helpful to have both string and number reference for this
  int  typecode;    // 0 = camera, 1 = sphere, 2 = plane, 3 = cylinder, 4 = quadric
  double width;
  double height;
  double color[3];
  double position[3];
  double normal[3];
  double center[3];
  double radius;
  A_J coeffs;       // quadric coefficients
  has_values flags; // help with error checking, flag to make sure values are set
} JSON_object ;

// This may not be the best approach, and it's certainly not most efficient - to have an array that
// is always 128 "JSON_object"s large. But it's clean and all of the data related to the JSON
// scene file is in this one struct, filehandle and all, that's what I like about it.
typedef struct JSON_file_struct {
  FILE* fh_in;
  int width;
  int height;
  int num_objects;
  JSON_object js_objects[128];
} JSON_file_struct ;

typedef double* V3;

// inline functions:
static inline double sqr (double v) {
  return v * v;
}

static inline void vNormalize (double* v) {
  double len = sqrt(sqr(v[0]) + sqr(v[1]) + sqr(v[2]));
  v[0] /= len;
  v[1] /= len;
  v[2] /= len;
}

static inline void vAdd(V3 a, V3 b, V3 c) {
  c[0] = a[0] + b[0];
  c[1] = a[1] + b[1];
  c[2] = a[2] + b[2];
}

static inline void vSubtract(V3 a, V3 b, V3 c) {
  c[0] = a[0] - b[0];
  c[1] = a[1] - b[1];
  c[2] = a[2] - b[2];
}

static inline double vDot(V3 a, V3 b) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static inline double vNorm(V3 a) {
  return sqrt(vDot(a,a));
}

static inline void vScale(V3 a, double s, V3 c) {
  c[0] = s * a[0];
  c[1] = s * a[1];
  c[2] = s * a[2];
}
// END inline functions

// global variables
int CURRENT_CHAR        = 'a';
int OUTPUT_MAGIC_NUMBER = 6; // default to P6 PPM format
int VERBOSE             = 0; // controls logfile message level
int ASCII_IMAGE         = 0; // controls if there is an ascii image printed to terminal while raycasting
int INFO                = 0; // controls if Info messages are printed, turn off prior to submission

// global data structures
JSON_file_struct    INPUT_FILE_DATA;
RGBPixel           *RGB_PIXEL_MAP;
PPM_file_struct     OUTPUT_FILE_DATA;
RGBAPixel          *RGBA_PIXEL_MAP;

// functions
void  writePPM        (char *outfile,         PPM_file_struct *input);
void  message         (char message_code[],   char message[]        );
void  writePPMHeader  (FILE* fh              );
void  reportPPMStruct (PPM_file_struct *input);
void  reportPixelMap  (RGBPixel *pm          );
void  checkJSON       (JSON_object *scene);
void  printJSONObjectStruct (JSON_object jostruct);
void  storeDouble           (int obj_count, char* key, double value);
void  storeVector           (int obj_count, char* key, double* value);
void  rayCast               (JSON_object *scene, RGBPixel *image);
double sphereIntersection   (double* Ro, double* Rd, double* C, double r);
double planeIntersection    (double* Ro, double* Rd, double* C, double* N);
double quadricIntersection  (double* Ro, double* Rd, double* C, A_J c);
double cylinderIntersection (double* Ro, double* Rd, double* C, double r);

unsigned char shadePixel (double value);

void  help();
int   computeDepth();
char* computeTuplType();
void  freeGlobalMemory ();
void  closeAndExit ();
void  reportScene();
double getCameraWidth();
double getCameraHeight();


/* 
 ------------------------------------------------------------------
 Parser and functions from Dr. P - refactored and enhanced
 ------------------------------------------------------------------
*/
int line = 1;

// nextC() wraps the getc() function and provides error checking and line
// number maintenance
int nextC(FILE* json) {
  int c = fgetc(json);
  if (VERBOSE) printf("'%c'", c);
  if (c == '\n') {
    line += 1;
  }
  if (c == EOF) {
    fprintf(stderr, "Error: Unexpected end of file on line number %d.\n", line);
    exit(1);
  }
  return c;
}

// expectC() checks that the next character is d.  If it is not it emits
void expectC(FILE* json, int d) {
  int c = nextC(json);
  if (c == d) return;
  fprintf(stderr, "Error: Expected '%c' on line %d.\n", d, line);
  exit(1);    
}


// skipWSpace() skips white space in the file.
void skipWSpace(FILE* json) {
  int c = nextC(json);
  while (isspace(c)) {
    c = nextC(json);
  }
  ungetc(c, json);
}


// nextString() gets the next string from the file handle and emits an error
// if a string can not be obtained.
char* nextString(FILE* json) {
  char buffer[129];
  int c = nextC(json);
  if (c != '"') {
    fprintf(stderr, "Error: Expected string on line %d.\n", line);
    exit(1);
  }  
  c = nextC(json);
  int i = 0;
  while (c != '"') {
    if (i >= 128) {
      fprintf(stderr, "Error: Strings longer than 128 characters in length are not supported.\n");
      exit(1);      
    }
    if (c == '\\') {
      fprintf(stderr, "Error: Strings with escape codes are not supported.\n");
      exit(1);      
    }
    if (c < 32 || c > 126) {
      fprintf(stderr, "Error: Strings may contain only ascii characters.\n");
      exit(1);
    }
    buffer[i] = c;
    i += 1;
    c = nextC(json);
  }
  buffer[i] = 0;
  if (VERBOSE) printf("\nnS:  returning string--> (%s)\n",buffer);
  return strdup(buffer);
}

// Get the next number, check that fscanf returns one item
double nextNumber(FILE* json) {
  double value;
  int rval;
  rval = fscanf(json, "%lf", &value);
  if (rval != 1) {
    fprintf(stderr, "Error: expected valid number on line %d, did not find one\n",line);
    exit(1);
  }
  if (VERBOSE) printf("\nnN:  returning number--> (%lf)(%d)\n",value,rval);
  return value;
}

double* nextVector(FILE* json) {
  double* v = malloc(4*sizeof(double));
  expectC(json, '[');
  skipWSpace(json);
  v[0] = nextNumber(json);
  skipWSpace(json);
  expectC(json, ',');
  skipWSpace(json);
  v[1] = nextNumber(json);
  skipWSpace(json);
  expectC(json, ',');
  skipWSpace(json);
  v[2] = nextNumber(json);
  skipWSpace(json);
  expectC(json, ']');
  if (VERBOSE) printf("nV: returning vector--> %d\n",v);
  return v;
}

// This is the big JSON parser function
void readScene(char* filename) {
  int c;
  int obj_count = 0;
  
  FILE* json = fopen(filename, "r");

  if (json == NULL) {
    fprintf(stderr, "Error: Could not open file \"%s\"\n", filename);
    exit(1);
  }
  
  skipWSpace(json);
  
  // Find the beginning of the list
  expectC(json, '[');

  skipWSpace(json);

  // Find all the objects in the JSON scene file
  int fail_safe = 0;
  while (1) {
    /* Error checking */
    // max supported objects * number of JSON lines per object (with margin of error)
    fail_safe++;
    if (fail_safe > 1280) message("Error","Do you have a ']' to terminate your JSON file?");
    if (obj_count > 128) message("Error","Maximum supported number of JSON objects is 128");

    /* Process file */
    c = fgetc(json);
    if (c == ']') {
      message("Error","No objects detected!\n");
      fclose(json);
      return;
    }
    if (c == '{') {
      skipWSpace(json);
      
      // Parse the object, getting the type first
      char* key = nextString(json);
      if (strcmp(key, "type") != 0) {
	fprintf(stderr, "Error: Expected \"type\" key on line number %d.\n", line);
	exit(1);
      }
      
      skipWSpace(json);
      expectC(json, ':');
      skipWSpace(json);
      
      // get the type of the object and store it at the index of the current object
      char* value = nextString(json);
      if (strcmp(value, "camera") == 0) {
	if (INFO) message("Info","Processing camera object...");
	INPUT_FILE_DATA.js_objects[obj_count].type = "camera";
	INPUT_FILE_DATA.js_objects[obj_count].typecode = 0;
	INPUT_FILE_DATA.num_objects = obj_count + 1;
      } else if (strcmp(value, "sphere") == 0) {
	if (INFO) message("Info","Processing sphere object...");
	INPUT_FILE_DATA.js_objects[obj_count].type = "sphere";
	INPUT_FILE_DATA.js_objects[obj_count].typecode = 1;
	INPUT_FILE_DATA.num_objects = obj_count + 1;
      } else if (strcmp(value, "plane") == 0) {
	if (INFO) message("Info","Processing plane object...");
	INPUT_FILE_DATA.js_objects[obj_count].type = "plane";
	INPUT_FILE_DATA.js_objects[obj_count].typecode = 2;
	INPUT_FILE_DATA.num_objects = obj_count + 1;
      } else if (strcmp(value, "cylinder") == 0) {
	if (INFO) message("Info","Processing cylinder object...");
	INPUT_FILE_DATA.js_objects[obj_count].type = "cylinder";
	INPUT_FILE_DATA.js_objects[obj_count].typecode = 3;
	INPUT_FILE_DATA.num_objects = obj_count + 1;
      } else if (strcmp(value, "quadric") == 0) {
	if (INFO) message("Info","Processing quadric object...");
	INPUT_FILE_DATA.js_objects[obj_count].type = "quadric";
	INPUT_FILE_DATA.js_objects[obj_count].typecode = 4;
	INPUT_FILE_DATA.num_objects = obj_count + 1;
      } else {
	fprintf(stderr, "Error: Unknown type, \"%s\", on line number %d.\n", value, line);
	exit(1);
      }
      char* type = value; // store for error checking later
      skipWSpace(json);
      
      // This while processes the attributes of the object
      while (1) {
	// , }
	c = nextC(json);
	if (c == '}') {
	  // stop parsing this object and increment the object counter
	  obj_count++;
	  break;
	} else if (c == ',') {
	  // read another field
	  skipWSpace(json);
	  char* key = nextString(json);
	  skipWSpace(json);
	  expectC(json, ':');
	  skipWSpace(json);
	  // read values
	  if ((strcmp(key, "width") == 0) ||
	      (strcmp(key, "height") == 0) ||
	      (strcmp(key, "radius") == 0) ||
	      (strcmp(key, "A") == 0) ||
	      (strcmp(key, "B") == 0) ||
	      (strcmp(key, "C") == 0) ||
	      (strcmp(key, "D") == 0) ||
	      (strcmp(key, "E") == 0) ||
	      (strcmp(key, "F") == 0) ||
	      (strcmp(key, "G") == 0) ||
	      (strcmp(key, "H") == 0) ||
	      (strcmp(key, "I") == 0) ||
	      (strcmp(key, "J") == 0)
	      ) {
	    double value = nextNumber(json);
	    // Error checking
	    if (strcmp(type,"sphere") == 0 && strcmp(key,"radius") != 0) message("Error","Sphere has extra value");
	    if (strcmp(type,"plane") == 0) message("Error","Plane has value that makes no sense(width,radius,etc)");
	    // store the value if pass checks
	    storeDouble(obj_count,key,value);
	  } else if ((strcmp(key, "color") == 0) ||
		     (strcmp(key, "position") == 0) ||
		     (strcmp(key, "normal") == 0)) {
	    double* value = nextVector(json);
	    // Error checking
	    if (strcmp(type,"sphere") == 0 && strcmp(key,"normal") == 0)
	      message("Error","Sphere shouldn't have normal");
	    // store the value if pass checks
	    storeVector(obj_count,key,value);
	    free(value);
	  } else {
	    fprintf(stderr, "Error: Unknown property, \"%s\", on line %d.\n",key, line);
	    exit(1);
	  }
	  skipWSpace(json);
	} else {
	  fprintf(stderr, "Error: Unexpected value on line %d\n", line);
	  exit(1);
	}
      }
      skipWSpace(json);
      c = nextC(json);
      if (c == ',') {
	skipWSpace(json);
      } else if (c == ']') {
	fclose(json);
	return;
      } else {
	fprintf(stderr, "Error: Expecting ',' or ']' on line %d.\n", line);
	exit(1);
      }
    }
  }
  if (INFO) message("Info","Read scene file");
}
/* 
 ------------------------------------------------------------------
 End pirated code - Parser and functions from Dr. P
 ------------------------------------------------------------------
*/


/*
 ------------------------------------------------------------------
                                 MAIN
 ------------------------------------------------------------------
*/
int main(int argc, char *argv[]) {
  // check for proper number of input args
  if (argc != 5) {
    help();
    return(1);
  }

  // process input arguments and report what is being processed, store some variables
  int width = atoi(argv[1]);
  int height = atoi(argv[2]);
  char *infile = argv[3];
  char *outfile = argv[4];
  if (strcmp(infile,outfile)  == 0) {printf("Error: input and output file names the same!\n"); return EXIT_FAILURE;}
  
  if (INFO) message("Info","Processing the following arguments:");
  if (INFO) printf("          Input : %s\n",infile);
  if (INFO) printf("          Output: %s\n",outfile);
  if (INFO) printf("          Width : %d\n",width);
  if (INFO) printf("          Height: %d\n",height);

  INPUT_FILE_DATA.width = width;
  INPUT_FILE_DATA.height = height;

  // parse the JSON
  //int parse_success = parseJSON(infile);
  // Read scene - code from class
  readScene(infile);
  
  // error checking
  checkJSON(INPUT_FILE_DATA.js_objects);

  // report results
  if (INFO) reportScene();

  // initialize the image buffer
  RGB_PIXEL_MAP = malloc(sizeof(RGBPixel) * INPUT_FILE_DATA.width * INPUT_FILE_DATA.height );
  
  // run the raycasting
  rayCast(INPUT_FILE_DATA.js_objects,RGB_PIXEL_MAP);

  // write the image
  writePPM(outfile,&OUTPUT_FILE_DATA);
  
  // prepare to exit
  freeGlobalMemory();
  return EXIT_SUCCESS;
}
/* 
 ------------------------------------------------------------------
                               END MAIN
 ------------------------------------------------------------------
*/



/*
  ------------------------
  FUNCTION IMPLEMENTATIONS
  ------------------------
*/

/*
  --- message ---
  - 9/10/16
  - rmr5
  ---------------
  print a message to stdout consisting of a message code and a message to a given channel
  current valid channels to write to (stdout, stderr, etc) - will add fh later
  //void message (char channel[], char message_code[], char message[]) {
*/
void message (char message_code[], char message[]) {
  if(message_code == "Error") {
    fprintf(stderr,"%s: %s\n",message_code,message);
    closeAndExit();
    exit(-1);
  } else {
    printf("%s: %s\n",message_code,message);
  }
}

/*
  --- help ---
  - rmr5
  ---------------
  print usage to user when arguments invalid
*/
void help () {
  message("Error","Invalid arguments!");
  message("Usage","raycast width height input.json output.ppm");
}

/*
  --- freeGlobalMemory ---
  - 9/15/16
  - rmr5
  ---------------
  free up any globally malloc memory
*/
// TODO: make this more universal
// TODO: this is causing core dumps for only certain cases, don't understand it
void freeGlobalMemory () {
  if (INFO) message("Info","Freeing global memory...");
  free(RGB_PIXEL_MAP);
  //free(RGBA_PIXEL_MAP);
  if (INFO) message("Info","Done.");
}

/*
  --- closeAndExit ---
  - 9/15/16
  - rmr5
  ---------------
  prepare to gracefully exit the program by freeing memory and closing any open filehandles
  TODO: need to finish
*/
void closeAndExit () {
  freeGlobalMemory();
  //fclose(INPUT_FILE_DATA->fh_in);
  exit(-1);
}


//  small helper to assign proper depth to a P7 file
int computeDepth() {
  if ((strcmp(OUTPUT_FILE_DATA.tupltype,"RGB_ALPHA")) == 0) {
    return 4; 
  } else {
    return 3;
  }
}

// helper to assign preper tupltype in P7
char* computeTuplType() {
  if ((strcmp(OUTPUT_FILE_DATA.tupltype,"RGB_ALPHA")) == 0) {
    if (VERBOSE) printf("cD: returning tupltype RGB_ALPHA because input was %s\n",OUTPUT_FILE_DATA.tupltype);
    return "RGB_ALPHA"; 
  } else {
    if (VERBOSE) printf("cD: returning tupltype RGB because input was %s\n",OUTPUT_FILE_DATA.tupltype);
    return "RGB"; 
  }
}

// helper function to write the header to a file handle
void writePPMHeader (FILE* fh) {
  int magic_number = OUTPUT_MAGIC_NUMBER;

  // These values/header elements are the same regardless format
  if (INFO) printf("Info: Converting to format %d ...\n",magic_number);
  fprintf(fh,"P%d\n",magic_number);
  fprintf(fh,"# PPM file format %d\n",magic_number);
  fprintf(fh,"# written by ppmrw(rmr5)\n");
  // make some variable assignments from input -> output
  OUTPUT_FILE_DATA.magic_number = magic_number;
  OUTPUT_FILE_DATA.width        = INPUT_FILE_DATA.width;
  OUTPUT_FILE_DATA.height       = INPUT_FILE_DATA.height;
  OUTPUT_FILE_DATA.alpha        = 255;
  
  if (magic_number == 3 || magic_number == 6) {
    fprintf(fh,"%d %d\n",       OUTPUT_FILE_DATA.width,OUTPUT_FILE_DATA.height);
    fprintf(fh,"%d\n",          OUTPUT_FILE_DATA.alpha);
  } else if (magic_number == 7) {
    OUTPUT_FILE_DATA.depth      = computeDepth();
    OUTPUT_FILE_DATA.tupltype   = computeTuplType();
    
    fprintf(fh,"WIDTH %d\n",    OUTPUT_FILE_DATA.width);
    fprintf(fh,"HEIGHT %d\n",   OUTPUT_FILE_DATA.height);
    fprintf(fh,"DEPTH %d\n",    OUTPUT_FILE_DATA.depth);
    fprintf(fh,"MAXVAL %d\n",   OUTPUT_FILE_DATA.alpha);
    fprintf(fh,"TUPLTYPE %d\n", OUTPUT_FILE_DATA.tupltype);
    fprintf(fh,"ENDHDR\n");
  } else {
    message("Error","Trying to output unsupported format!\n");
  }
  if (INFO) message("Info","Done writing header");
}

/*
  --- writePPM ---
  - 9/13/16
  - rmr5
  ---------------
  Major function to write the actual output ppm file
  takes a output filename and an input PPM struct
  uses global data

  This function has case statements to support all supported formats 
*/
void writePPM (char *outfile, PPM_file_struct *input) {
  if (INFO) printf("Info: Writing file %s...\n",outfile);
  FILE* fh_out = fopen(outfile,"wb");

  // -------------------------- write header ---------------------------------
  writePPMHeader(fh_out);
  // ---------------------- done write header --------------------------------

  // -------------------------- write image ----------------------------------
  int pixel_index = 0;
  int modulo;
  switch(OUTPUT_FILE_DATA.magic_number) {
    // P3 format
    // Iterate over each pixel in the pixel map and write them byte by byte
  case(3):
    if (INFO) message("Info","Outputting format 3");
    while(pixel_index < (OUTPUT_FILE_DATA.width) * (OUTPUT_FILE_DATA.height)) {      
      fprintf(fh_out,"%3d %3d %3d",RGB_PIXEL_MAP[pixel_index].r,RGB_PIXEL_MAP[pixel_index].g,RGB_PIXEL_MAP[pixel_index].b);
      modulo = (pixel_index + 1) % (OUTPUT_FILE_DATA.width);
      if ( modulo == 0 ) {
	fprintf(fh_out,"\n");
      } else {
	fprintf(fh_out," ");
      }
      pixel_index++;
    }
    break;
    // P6 format
    // write the entire pixel_map in one command
  case(6):
    if (INFO) message("Info","Outputting format 6");
    fwrite(RGB_PIXEL_MAP, sizeof(RGBPixel), OUTPUT_FILE_DATA.width * OUTPUT_FILE_DATA.height, fh_out);
    break;
    // P7 format
  case(7):
    // write the entire pixel_map in one command, RGB writes from RGB pixel_map and RGBA writes from RGBA pixel_map
    if (INFO) message("Info","Outputting format 7");
    if (strcmp(OUTPUT_FILE_DATA.tupltype,"RGB_ALPHA") == 0) {
      if (INFO) message("Info","   output file will have alpha data");
      fwrite(RGBA_PIXEL_MAP, sizeof(RGBAPixel), OUTPUT_FILE_DATA.width * OUTPUT_FILE_DATA.height, fh_out);
    } else {
      if (INFO) message("Info","   output file is RGB only");
      fwrite(RGB_PIXEL_MAP, sizeof(RGBPixel), OUTPUT_FILE_DATA.width * OUTPUT_FILE_DATA.height, fh_out);
    }
    break;
  default:
    message("Error","Unrecognized output format");
  }
  // ---------------------- done write image ---------------------------------

  fclose(fh_out);
  if (INFO) reportPPMStruct(&OUTPUT_FILE_DATA);
  if (INFO) message("Info","Done writing PPM");
}

// helper function to visualize what's in a given PPM struct
void reportPPMStruct (PPM_file_struct *input) {
  message("Info","Contents of PPM struct:");
  printf("     magic_number: %d\n",input->magic_number);
  printf("     width:        %d\n",input->width);
  printf("     height:       %d\n",input->height);
  if (input->magic_number == 7) {
    printf("     max_value:    %d\n",input->alpha);
    printf("     depth:        %d\n",input->depth);
    printf("     tupltype:     %s\n",input->tupltype);
  } else {
    printf("     alpha:        %d\n",input->alpha);
  }
}

// small utility function to print the contents of a pixelMap
void reportPixelMap (RGBPixel *pm) {
  int index = 0;
  int fail_safe = 0;
  while(index < sizeof(pm) && fail_safe < 1000) {
    printf("rPM: [%d] = [%d,%d,%d]\n",index,pm[index].r,pm[index].g,pm[index].b);
    index++;
    fail_safe++;
  }
}

// helper to print out a JSON_object
void printJSONObjectStruct (JSON_object jostruct) {
  printf("type: %s\n",jostruct.type);
  if (strcmp(jostruct.type,"camera") == 0) {
    printf(" width: %f\n",jostruct.width);
    printf("height: %f\n",jostruct.height);
  } else if (strcmp(jostruct.type,"sphere") == 0) {
    printf("    color: [%f, %f, %f]\n",jostruct.color[0], jostruct.color[1], jostruct.color[2]);
    printf(" position: [%f, %f, %f]\n",jostruct.position[0], jostruct.position[1], jostruct.position[2]);
    printf("   radius: %f\n",jostruct.radius);
  } else if (strcmp(jostruct.type,"plane") == 0) {
    printf("    color: [%f, %f, %f]\n",jostruct.color[0], jostruct.color[1], jostruct.color[2]);
    printf(" position: [%f, %f, %f]\n",jostruct.position[0], jostruct.position[1], jostruct.position[2]);
    printf("   normal: [%f, %f, %f]\n",jostruct.normal[0], jostruct.normal[1], jostruct.normal[2]);
  } else if (strcmp(jostruct.type,"quadric") == 0) {
    printf("    color: [%f, %f, %f]\n",jostruct.color[0], jostruct.color[1], jostruct.color[2]);
    printf(" position: [%f, %f, %f]\n",jostruct.position[0], jostruct.position[1], jostruct.position[2]);
    printf("        A: %f\n",jostruct.coeffs.A);
    printf("        B: %f\n",jostruct.coeffs.B);
    printf("        C: %f\n",jostruct.coeffs.C);
    printf("        D: %f\n",jostruct.coeffs.D);
    printf("        E: %f\n",jostruct.coeffs.E);
    printf("        F: %f\n",jostruct.coeffs.F);
    printf("        G: %f\n",jostruct.coeffs.G);
    printf("        H: %f\n",jostruct.coeffs.H);
    printf("        I: %f\n",jostruct.coeffs.I);
    printf("        J: %f\n",jostruct.coeffs.J);
  } else {
    printf("Error: unrecognized type\n");
  }
  printf("\n");
}

// helper to report the results of a scene parse
void reportScene () {
  int len_array = INPUT_FILE_DATA.num_objects;
  if (INFO) printf("\n\n---------------------\n");
  if (INFO) message("Info","PARSE RESULTS:");
  if (INFO) printf("---------------------\n");
  if (INFO) printf("Processed scene with %d objects:\n\n",len_array);
  for (int i = 0; i < len_array; i++) {
    printJSONObjectStruct(INPUT_FILE_DATA.js_objects[i]);
  }
}

// helper to store a double onto our JSON object file
void storeDouble(int obj_count, char* key, double value) {
  if (VERBOSE) printf("   sD: storing %s,%lf at %d\n",key,value,obj_count);

  // store the actual value, not sure how to say ".key" and get it to evaluate key so need these ifs
  if (strcmp(key,"width") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].width = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_width = 1;
  } else if (strcmp(key,"height") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].height = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_height = 1;
  } else if (strcmp(key,"radius") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].radius = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_radius = value;
  } else if (strcmp(key, "A") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.A = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_A = 1;
  } else if (strcmp(key, "B") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.B = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_B = 1;
  } else if (strcmp(key, "C") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.C = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_C = 1;
  } else if (strcmp(key, "D") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.D = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_D = 1;
  } else if (strcmp(key, "E") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.E = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_E = 1;
  } else if (strcmp(key, "F") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.F = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_F = 1;
  } else if (strcmp(key, "G") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.G = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_G = 1;
  } else if (strcmp(key, "H") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.H = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_H = 1;
  } else if (strcmp(key, "I") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.I = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_I = 1;
  } else if (strcmp(key, "J") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].coeffs.J = value;
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_J = 1;
  } else {
    // This should never happen
    message("Error","Interally trying to store unknown key type");
  }
}

// helper to store a vector onto our JSON object file
void storeVector(int obj_count, char* key, double* value) {
  if (VERBOSE) printf("   sV: storing %s at %d\n",key,obj_count);
  if (strcmp(key,"color") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].color[0] = value[0];
    INPUT_FILE_DATA.js_objects[obj_count].color[1] = value[1];
    INPUT_FILE_DATA.js_objects[obj_count].color[2] = value[2];
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_color = 1;
  } else if (strcmp(key,"position") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].position[0] = value[0];
    INPUT_FILE_DATA.js_objects[obj_count].position[1] = value[1];
    INPUT_FILE_DATA.js_objects[obj_count].position[2] = value[2];
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_position = 1;
  } else if (strcmp(key,"normal") == 0) {
    INPUT_FILE_DATA.js_objects[obj_count].normal[0] = value[0];
    INPUT_FILE_DATA.js_objects[obj_count].normal[1] = value[1];
    INPUT_FILE_DATA.js_objects[obj_count].normal[2] = value[2];
    INPUT_FILE_DATA.js_objects[obj_count].flags.has_normal = 1;
  } else {
    // This should never happen
    message("Error","Interally trying to store unknown vector key type");
  }

}

// Raycaster function
// builds the image based on a scene
void  rayCast(JSON_object *scene, RGBPixel *image) {

  ////////////
  // variables
  ////////////
  int intersect = 0; // dummy var while intersection tests not working
  // number of pixels that represent height/width
  int M = INPUT_FILE_DATA.height;
  int N = INPUT_FILE_DATA.width;
  int pixmap_length = M * N;
  // this represents the center of the view plane
  double cx = 0;
  double cy = 0;
  double cz = 1;
  // make a view plane according to the (first) camera object in the JSON
  double w = getCameraWidth();
  double h = getCameraHeight();
  // height/width of each pixel
  double pixwidth = w / N;
  double pixheight = h / M;
  int i = 0; // pixelmap counter, since my pixelmap is a flat array
  
  if (INFO) printf("Raycasting %d x %d image to memory ...\n",N,M);

  //////////////////////////////////////
  // copy of psuedo code from text/class
  //////////////////////////////////////
  //  for (int y = 0; y < M; y += 1) {
  for (int y = M; y >= 0; y -= 1) { // y-axis was flipped, so run this backwards
    for (int x = 0; x < N; x += 1) {
      // origin
      double Ro[3] = {0,0,0}; // vector that represents a point that represents the origin
      // direction
      // Rd = normalize(P - Ro), origin in 0 so skip that, but need to normalize
      // this won't work prior to C 1999 to evaluate in the static initializer
      double Rd[3] = {
	cx - (w/2) + pixwidth * ( x + 0.5),
	cy - (h/2) + pixheight * ( y + 0.5),
	cz
      };

      // next, need to make Rd so that it's actually normalized
      vNormalize(Rd);

      // structure of every ray tracer you will ever encounter
      // go over all x/y values for a scene and check for intersections
      double best_t = INFINITY;
      int    best_t_index = 129;

      for (int o = 0; o < INPUT_FILE_DATA.num_objects; o += 1) {
	//printf("DBG: o(%d) against no(%d)\n",o,INPUT_FILE_DATA.num_objects);
	// t stores if we have an intersection or not
	double t = 0;

	switch(INPUT_FILE_DATA.js_objects[o].typecode) {
	case 0:
	  //if (INFO) message("Info","Skipping camera object...");
	  break;
	case 1:
	  //if (INFO) message("Info","processing sphere...");
	  t = sphereIntersection(Ro,Rd,
				 INPUT_FILE_DATA.js_objects[o].position,
				 INPUT_FILE_DATA.js_objects[o].radius);
	  break;
	case 2:	
	  //if (INFO) message("Info","processing plane...");
	  t = planeIntersection(Ro,Rd,
				 INPUT_FILE_DATA.js_objects[o].position,
				 INPUT_FILE_DATA.js_objects[o].normal);
	  break;
	case 3:
	  //	  t = cylinder_intersection(Ro,Rd,objects[o]->cylinder.center,objects[o]->cylinder.radius);
	  t = cylinderIntersection(Ro,Rd,
				   INPUT_FILE_DATA.js_objects[o].position,
				   INPUT_FILE_DATA.js_objects[o].radius);
	  break;
	case 4:
	  t = quadricIntersection(Ro,Rd,
				   INPUT_FILE_DATA.js_objects[o].position,
				   INPUT_FILE_DATA.js_objects[o].coeffs);
	  break;
	default:
	  message("Error","Unhandled typecode, camera/plane/sphere are supported");
	}
	if (t > 0 && t < best_t) {
	  best_t = t;
	  best_t_index = o;
	}
      }
      // Now look at the t value and see if there was an intersection
      // remember that you could have multiple objects in from of each other, check for the smaller of the
      // t values, that hit first, color that one
      if (best_t > 0 && best_t != INFINITY) {
	if (ASCII_IMAGE) printf("#");
	RGB_PIXEL_MAP[i].r = shadePixel(scene[best_t_index].color[0]);
	RGB_PIXEL_MAP[i].g = shadePixel(scene[best_t_index].color[1]);
	RGB_PIXEL_MAP[i].b = shadePixel(scene[best_t_index].color[2]);
      } else {
	if (ASCII_IMAGE) printf(".");
	RGB_PIXEL_MAP[i].r = shadePixel(DEFAULT_COLOR);
	RGB_PIXEL_MAP[i].g = shadePixel(DEFAULT_COLOR);
	RGB_PIXEL_MAP[i].b = shadePixel(DEFAULT_COLOR);
      }
      i++; // increment the pixelmap counter
    }
    if (ASCII_IMAGE) printf("\n");
  }
  if (VERBOSE) message("Info","Done raycasting the image");
}

// helper function to convert 0 to 1 color scale into 0 to 255 color scale for PPM
unsigned char shadePixel (double value) {
  if (value > 1.0) {
    message("Error","Unsupported max color value, expected between 0 and 1.0");
  } else {
    return round(value * 255);
  }
}

/////////////////////////
// Intersection checkers
/////////////////////////
// Step 1. Find the equation for the object we are interested in
// Step 2. Parameterize the equation with a center point if needed
// Step 3. Substitute the equation for ray into our object equation
// Step 4. Solve for t.
//      4a. Rewrite the equation (flatten, get rid of parens). (maple/mathmatica will solve this, or algebra)
//      4b. rewrite the equation in terms of t, want to solve for t
// Step 5. Use the quadratic equation (if relevant) to solve for t
/////////////////////////

// Sphere intersection code (from http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter1.htm)
double sphereIntersection(double* Ro, double* Rd, double* C, double r) {
  // Xhit = pr + (tclose - a)*ur from the notes, same as
  // S = the set of points[xs, ys, zs], where (xs - xc)2 + (ys - yc)2 + (zs - zc)2 = r^2
  // (Ro[0] + Rd[0] * t - C[0])2 + (Ro[1] + Rd[1] * t - C[1])2 + (Ro[2] + Rd[2] * t - C[2])2 = r^2
  // or A*t^2 + B*t + C = 0
  double a = sqr(Rd[0]) + sqr(Rd[1]) + sqr(Rd[2]); // with normalized ray, should always be 1
  double b = 2 * (Rd[0] * (Ro[0] - C[0]) + Rd[1] * (Ro[1] - C[1]) + Rd[2] * (Ro[2] - C[2]));
  double c = sqr(Ro[0] - C[0]) + sqr(Ro[1] - C[1]) + sqr(Ro[2] - C[2]) - sqr(r);

  double disc = sqr(b) - 4 * a * c;
  if (disc < 0) return -1;

  // take the square root
  disc = sqrt(disc);
  
  //t0, t1 = (- B + (B^2 - 4*C)^1/2) / 2 where t0 is for (-) and t1 is for (+)
  // compute t0, if positive, this is the smaller of 2 intersections, we are done, return it
  double t0 = (-b - disc) / (2 * a);
  if (t0 > 0) return t0;

  double t1 = (-b + disc) / (2 * a);
  if (t1 > 0) return t1;

  // no intersection if we fall through to here
  return -1;
}

// plane intersection code (from several sources)
// arguments are: the ray (origin/direction), center of the plane, normal of the plane
double planeIntersection(double* Ro, double* Rd, double* C, double* N) {

  // if Vd = (Pn � Rd) = 0, no intersection, so compute it first and return if no intersection
  double Vd = vDot(N,Rd);
  if (Vd == 0) return -1;

  // Now subtract ray origin from the point on the plane and dot it with normal
  double the_diff[3];
  vSubtract(C,Ro,the_diff);
  double V0 = vDot(N,the_diff);

  VERBOSE = 1; // DBG TODO remove
  //if (VERBOSE) printf("C0: %f, C1: %f, C2: %f\n",the_diff[0],the_diff[1],the_diff[2]);

  double t = V0 / Vd;
  //  if (VERBOSE) printf("V0: %f, Vd: %f, t: %f\n",V0,Vd,t);

  if (t < 0) return -1; // plane intersection is behind origin, ignore it

  // if we got this far, plane intersects the ray in front of origin
  //  if (VERBOSE) printf("returning %f\n",t);
  return t;
}

// Cylinder intersection code (from example in class) 
double cylinderIntersection(double* Ro, double* Rd, double* C, double r) {
  // x^2 + z^2 = r^2   using z instead of y will make it go up/down, instead of looking head on
  double a = (sqr(Rd[0]) + sqr(Rd[2])); // remember that x/y/z are in array form
  double b = (2 * (Ro[0] * Rd[0] - Rd[0] * C[0] + Ro[2] * Rd[2] - Rd[2] * C[2]));
  double c = sqr(Ro[0]) - 2*Ro[0] * C[0] + sqr(C[0]) + sqr(Ro[2]) - 2*Ro[2] * C[2] + sqr(C[2]) - sqr(r);

  // discriminant, remember the negative version is not real, imaginary, not 
  double disc = sqr(b) - 4 * a * c;
  if (disc < 0 ) return -1; // this is the signal that there was not an intersection

  // since we are using it more than once
  disc = sqrt(disc);
  
  double t0 = (-b - disc) / (2*a);
  if (t0 > 0) return t0; // smaller/lesser of the 2 values, needs to come first

  double t1 = (-b + disc) / (2*a);
  if (t1 > 0) return t1;

  // this is a default case if we have no intersection, could also just return t1, but this accounts
  // for "numeric stability" as numbers become very close to 0
  return -1;
}

// Quadric intersection code
double quadricIntersection(double* Ro, double* Rd, double* C, A_J c) {
  // Based on the siggraph documentation on
  // http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter4.htm
  // Had to add in the offsets to position the object, the siggraph page didn't have that
  //
  // Hand-solved the equation that has the offsets
  // into the Aq*t^2 + Bq*t + Cq = 0 format for quadratic solving
  double Aq = c.A*sqr(Rd[0]) + c.B*sqr(Rd[1]) + c.C*sqr(Rd[2]) + c.D*Rd[0]*Rd[1] + c.E*Rd[0]*Rd[2] + c.F*Rd[1]*Rd[2];
  double Bq = - 2*c.A*Rd[0]*C[0] - 2*c.B*Rd[1]*C[1] - 2*c.C*Rd[2]*C[2] - c.D*Rd[0]*C[1] - c.D*Rd[1]*C[0] -
    c.E*Rd[0]*C[2] - c.E*Rd[2]*C[0] - c.F*Rd[1]*C[2] - c.F*Rd[2]*C[1] + c.G*Rd[0] + c.H*Rd[1] + c.I*Rd[2];
  double Cq = c.A*sqr(C[0]) + c.B*sqr(C[1]) + c.C*sqr(C[2]) + c.D*C[0]*C[1] +
    c.E*C[0]*C[2] + c.F*C[1]*C[2] - c.G*C[0] - c.H*C[1] - c.I*C[2] + c.J;

  // Some debug statements
  if (VERBOSE) printf("DBG : xyz=(%f,%f,%f) AJ=(%f,%f,%f,%f)\n",C[0],C[1],C[2],c.A,c.B,c.C,c.D);
  if (VERBOSE) printf("DBG : Rd's > x=%f, y=%f, z=%f\n",Rd[0],Rd[1],Rd[2]);
  if (VERBOSE) printf("DBG : Aq=%f, Bq=%f, Cq=%f)\n",Aq,Bq,Cq);
  
  // 1. Check Aq = 0 (If Aq = 0 then t = -Cq / Bq
  // 2.If Aq � 0, then check the discriminant (If Bq2 - 4AqCq < 0 then there is no intersection)
  // 3. Compute t0 and if t0 > 0 then done else compute t1
  if (Aq == 0) return -Cq / Bq;
  
  // discriminant 
  double disc = (sqr(Bq) - (4 * Aq * Cq));
  if (VERBOSE) printf("DBG : disc=(%f), where Bq^2=(%f) and 4ac=(%f)\n",disc,sqr(Bq),(4*Aq*Cq));
  if (disc < 0) return -1; // no intersection in this case
  
  // since we are using it more than once
  disc = sqrt(disc) ;
  
  double t0 = (-Bq - disc) / (2 * Aq);
  if (VERBOSE) printf("DBG : t0=%f\n",t0);
  if (t0 > 0) return t0; // smaller/lesser of the 2 values, needs to come first

  double t1 = (-Bq + disc) / (2 * Aq);
  if (VERBOSE) printf("DBG : t1=%f\n",t1);
  if (t1 > 0) return t1;

  return -1;
}

// Helper functions to find the first camera object and get it's specified width/height
double getCameraWidth() {
  double w = 0.0;
  for (int o = 0; o < INPUT_FILE_DATA.num_objects; o++) {
    if (INPUT_FILE_DATA.js_objects[o].typecode == 0) {
      w = INPUT_FILE_DATA.js_objects[o].width;
      if (w > 0) {
	if (INFO) printf("Info: Found camera object width %f\n",w);
	return w;
      } else {
	message("Error","Unsupported camera width less than zero");
      }
    }
  }
  return w;
}
double getCameraHeight() {
  double h = 0.0;
  for (int o = 0; o < INPUT_FILE_DATA.num_objects; o++) {
    if (INPUT_FILE_DATA.js_objects[o].typecode == 0) {
      h = INPUT_FILE_DATA.js_objects[o].height;
      if (h > 0) {
	if (INFO) printf("Info: Found camera object height %f\n",h);
	return h;
      } else {
	message("Error","Unsupported camera height less than zero");
      }
    }
  }
  return h;
}

// helper function for JSON error checking (like does a sphere have a width, etc...)
void checkJSON (JSON_object *object) {
  if (INFO) message("Info","Checking JSON for errors...");
  // variables
  
  // code body
  for (int o = 0; o < INPUT_FILE_DATA.num_objects; o++) {
    switch(object[o].typecode) {
    case 0: // camera
      if (!object[o].width || !object[o].height)
	message("Error","Camera object must have width and height properties");
      //      if (object[o].radius || sizeof(object[o].normal) > 0 || sizeof(object[o].color) > 0)
      if (object[o].radius)
	if (INFO) message("Info","Ignoring camera object properties in excess of width/height");
      break;
    case 1: // sphere
      if (!object[o].radius)      
	message("Error","Sphere object is missing radius or is zero!");
      if (!object[o].flags.has_position)      
	message("Error","Sphere object is missing position!");
      if (!object[o].flags.has_color)      
	message("Error","Sphere object is missing color!");
      if (object[o].width || object[o].height || sizeof(object[o].normal) > 0)
	if (INFO) message("Info","Ignoring sphere object properties in excess of radius/position/color");
      break;
    case 2: // plane
      if (!object[o].flags.has_position)      
	message("Error","Plane object is missing position!");
      if (!object[o].flags.has_color)      
	message("Error","Plane object is missing color!");
      if (!object[o].flags.has_normal)      
	message("Error","Plane object is missing normal!");
      break;
    case 3: // cylinder
      break;
    case 4:
      if (!object[o].flags.has_position)      
	message("Error","Quadric object is missing position!");
      if (!object[o].flags.has_color)      
	message("Error","Quadric object is missing color!");
      if (!object[o].flags.has_A)      
	message("Error","Quadric object is missing A parameter!");
      if (!object[o].flags.has_B)      
	message("Error","Quadric object is missing B parameter!");
      if (!object[o].flags.has_C)      
	message("Error","Quadric object is missing C parameter!");
      if (!object[o].flags.has_D)      
	message("Error","Quadric object is missing D parameter!");
      if (!object[o].flags.has_E)      
	message("Error","Quadric object is missing E parameter!");
      if (!object[o].flags.has_F)      
	message("Error","Quadric object is missing F parameter!");
      if (!object[o].flags.has_G)      
	message("Error","Quadric object is missing G parameter!");
      if (!object[o].flags.has_H)      
	message("Error","Quadric object is missing H parameter!");
      if (!object[o].flags.has_I)      
	message("Error","Quadric object is missing I parameter!");
      if (!object[o].flags.has_J)      
	message("Error","Quadric object is missing J parameter!");
      break;
    default:
      message("Error","Un-caught error, was missed during parsing");
    }
  }
  if (INFO) message("Info","Done checking JSON for errors...");
}
