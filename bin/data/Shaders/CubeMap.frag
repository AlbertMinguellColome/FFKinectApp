#version 120

uniform samplerCube envMap;
uniform float reflectivity;

varying vec3 reflectVec;

void main()
{
    
    vec4 cubeColor = textureCube(envMap, vec3(reflectVec.x, -reflectVec.y, reflectVec.z));
    
    gl_FragColor = vec4(0.5, 0.5, 0.5, 1.0);
    gl_FragColor.xyz = mix( gl_FragColor.xyz, cubeColor.xyz, reflectivity );
}
