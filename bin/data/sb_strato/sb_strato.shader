//**********************************************************************//
//									//
//	stratosphere.shader for Q3Radiant				//
//	by Sock - 23rd November 2000					//
//									//

// Direction & elevation checked and adjusted - Speaker
//**********************************************************************//

textures/skies/sb_strato
{
	qer_editorimage env/sb_strato/stratosphere_ft.tga
	surfaceparm noimpact
	surfaceparm nolightmap
	q3map_globaltexture
	q3map_lightsubdivide 256
	q3map_surfacelight 50
	surfaceparm sky
	q3map_sun 0.941 0.984 1.00 150 270 50
	skyparms env/sb_strato/stratosphere - -
}
