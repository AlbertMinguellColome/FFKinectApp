#version 120

varying vec3 reflectVec;
uniform float displacementAmount;

vec4 displaceFatternFunc( vec4 pos, vec3 norm )
{
    vec4 new_pos;
    vec3 new_norm;
    float multiplier;
    
    multiplier = 8.5;
    
    new_pos = pos;
    /*new_norm.x = norm.x * multiplier;
    new_norm.y = norm.y * multiplier;
    new_norm.z = norm.z * multiplier;*/
    
    new_pos.xyz += norm * multiplier;
    
    //new_norm.xyz =
//new_norm.xyz = norm *
  //  float l = sqrt((new_norm.x*new_norm.x) + (new_norm.y*new_norm.y) + (new_norm.z*new_norm.z));
  // vec3 NewN = normalize(
   /* new_pos.x += new_norm.x/l;
    new_pos.y += new_norm.y/l;
    new_pos.z += new_norm.z/l;
    new_pos.w = pos.*/
    
    
//    new_pos.x = pos.x ;
//    new_pos.y = pos.y ;
//    new_pos.z = pos.z ;
//    new_pos.w = pos.w;
    
    return new_pos;
}
vec4 displaceVertexFunc( vec4 pos, float phase, float frequency )
{
    vec4 new_pos;
    
    new_pos.x = pos.x;
    new_pos.z = pos.z;
    new_pos.w = pos.w;
    
    float dist = sqrt(pos.x*pos.x + pos.z*pos.z);
    new_pos.y = pos.y + 20.0 * sin( frequency * dist + phase );
    
    return new_pos;
}
void main()
{
    vec3 V = vec3(gl_ModelViewMatrix * gl_Vertex);
    vec3 N = normalize(gl_NormalMatrix * gl_Normal);
    reflectVec = reflect(normalize(-V), N);
    
    vec4 displacedPosition;
    displacedPosition = displaceVertexFunc(gl_Vertex, 2.0, 50 );
  //  gl_Position = gl_ModelViewProjectionMatrix * displacedPosition;//gl_Vertex;
    
    vec4 newVertexPos;
    newVertexPos = vec4(gl_Normal * displacementAmount, 0.0) + gl_Vertex;
   // gl_Position = gl_ModelViewProjectionMatrix * newVertexPos;
    
    

    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
