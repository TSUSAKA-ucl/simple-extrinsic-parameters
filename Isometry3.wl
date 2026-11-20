BeginPackage["Isometry3`", {"Quaternions`"}]
rotx::usage="rotation matrix around X axis"
roty::usage="rotation matrix around Y axis"
rotz::usage="rotation matrix around Z axis"
translate::usage="translation"
quat2mat::usage="rotation matrix from Quaternion"
mat2quat::usage="Quaternion from rotation matrix"
eulerZYX::usage="roll pitch yaw from rotation matrix"
eulerXYZ::usage="rotx.roty.rotz from rotation matrix"
(* *)
Begin["`Private`"]
(* *)
rotx[t_]:={{1,0,0,0},{0,Cos[t],-Sin[t],0},{0,Sin[t],Cos[t],0},{0,0,0,1}}
roty[t_]:={{Cos[t],0,Sin[t],0},{0,1,0,0},{-Sin[t],0,Cos[t],0},{0,0,0,1}}
rotz[t_]:={{Cos[t],-Sin[t],0,0},{Sin[t],Cos[t],0,0},{0,0,1,0},{0,0,0,1}}
translate[p_]:={{1,0,0,p[[1]]},{0,1,0,p[[2]]},{0,0,1,p[[3]]},{0,0,0,1}}
quat2mat[q_]:={{q[[2]]^2 - q[[3]]^2 - q[[4]]^2 + q[[1]]^2,
		  2(q[[2]] q[[3]] - q[[4]] q[[1]]),
		  2(q[[2]] q[[4]] + q[[3]] q[[1]]), 0},
	       {2 (q[[2]] q[[3]] + q[[4]] q[[1]]),
		-q[[2]]^2 + q[[3]]^2 - q[[4]]^2 + q[[1]]^2,
		2(q[[3]] q[[4]] - q[[2]] q[[1]]), 0},
	       {2(q[[2]] q[[4]] - q[[3]] q[[1]]),
		2(q[[3]] q[[4]] + q[[2]] q[[1]]),
		-q[[2]]^2 - q[[3]]^2 + q[[4]]^2 + q[[1]]^2, 0},
		{0,0,0,1}}
mat2quat[m_]:=With[{w4 = 2Sqrt[m[[1,1]]+m[[2,2]]+m[[3,3]]+1]},
		    Quaternion[w4/4,
			       (m[[3,2]]-m[[2,3]])/w4,
			       (m[[1,3]]-m[[3,1]])/w4,
			       (m[[2,1]]-m[[1,2]])/w4]]
			       
eulerZYX[m_]:=Module[{nx,ox,ax,ny,oy,ay,nz,oz,az},
		     {{nx,ox,ax},{ny,oy,ay},{nz,oz,az}} = m[[1;;3,1;;3]];
		     If[(nx^2+ny^2)==0,
			 {0, Pi/2, ArcTan[oy, oz]},
			 {ArcTan[nx, ny],
			  ArcTan[(nx^2+ny^2), - nz Sqrt[nx^2+ny^2]],
			  ArcTan[- ny ox + nx oy, ny ax - nx ay]
			 }
			]]
eulerXYZ[m_]:=Module[{nx,ox,ax,ny,oy,ay,nz,oz,az},
		     {{nx,ox,ax},{ny,oy,ay},{nz,oz,az}} = m[[1;;3,1;;3]];
		     If[(nx^2+ox^2)==0,
			 {0, Pi/2, ArcTan[oy, ay]},
			 {
			   ArcTan[az, -ay],
			   ArcTan[(az^2+ay^2), ax Sqrt[az^2+ay^2]],
			   ArcTan[-ay oz + az oy, - ay nz + az ny]
			 }
			]]
End[]
EndPackage[]
