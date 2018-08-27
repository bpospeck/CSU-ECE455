#include <math.h>
#include "vector_math.h"

h_vector *hvarray(nl,nh)
int nl,nh;
{
        h_vector *v;

        v=(h_vector *)malloc((unsigned) (nh-nl+1)*sizeof(h_vector));
        if (!v) nrerror("allocation failure in hvarray()");
        return v-nl;
}

h_transform *htarray(nl,nh)
int nl,nh;
{
        h_transform *v;

        v=(h_transform *)malloc((unsigned) (nh-nl+1)*sizeof(h_transform));
        if (!v) nrerror("allocation failure in htarray()");
        return v-nl;
}

/* allocate space for an array of homogeneous vectors with */
/* indices from nl to nh (see nrutil.c in numerical recipes) */
/*
h_vector **hvarray(nl,nh) 
int nl,nh;
{
	int i;
        h_vector **m;

        m=(h_vector **) malloc((unsigned) (nh-nl+1)*sizeof(h_vector*));
        if (!m) nrerror("allocation failure 1 in hvarray()");
        m -= nl;

        for(i=nl;i<=nh;i++) {
                m[i]=(h_vector *) malloc((unsigned) sizeof(h_vector));
                if (!m[i]) nrerror("allocation failure 2 in hvarray()");
        }
        return m;
}
*/
/* allocate space for an array of homogeneous vectors with */
/* indices from nl to nh (see nrutil.c in numerical recipes) */
/*
h_transform **htarray(nl,nh) 
int nl,nh;
{
	int i;
        h_transform **m;

        m=(h_transform **) malloc((unsigned) (nh-nl+1)*sizeof(h_transform*));
        if (!m) nrerror("allocation failure 1 in htarray()");
        m -= nl;

        for(i=nl;i<=nh;i++) {
                m[i]=(h_transform *) malloc((unsigned) sizeof(h_transform));
                if (!m[i]) nrerror("allocation failure 2 in htarray()");
        }
        return m;
}
*/

float dot(v1, v2)  /* dot product of two vectors */
h_vector v1;
h_vector v2;
{
	return(v1.x*v2.x + v1.y*v2.y + v1.z*v2.z);
}

h_vector cross(v1,v2) /* cross product of two vectors */
h_vector v1;
h_vector v2;
{
	h_vector ortho;

	ortho.x=v1.y*v2.z-v1.z*v2.y;
	ortho.y=v1.z*v2.x-v1.x*v2.z;
	ortho.z=v1.x*v2.y-v1.y*v2.x;
	ortho.w=1;

	return(ortho);
}

h_vector add_vector(v1,v2) /* add two vectore */
h_vector v1;
h_vector v2;
{
	h_vector sum;

	sum.x=v1.x+v2.x;
	sum.y=v1.y+v2.y;
	sum.z=v1.z+v2.z;
	sum.w=1;

	return(sum);
}

h_vector scale_vector(s,v) /* scale a vector */
float s;
h_vector v;
{
	h_vector scaled;

	scaled.x=s*v.x;
	scaled.y=s*v.y;
	scaled.z=s*v.z;
	scaled.w=1;

	return(scaled);
}

float vector_norm(v) /* return vector norm (length) */
h_vector v;
{
	return(sqrt(dot(v,v)));
}

h_vector unit_vector(v) /* return a unit vector */
h_vector v;
{
	return(scale_vector(1/vector_norm(v),v));
}

h_vector multTxV(T,v) /* multiply a transform and a vector */
h_transform T;
h_vector v;
{
        h_vector result;
 
        result.x=v.x*T.n.x+v.y*T.o.x+v.z*T.a.x+v.w*T.p.x;
        result.y=v.x*T.n.y+v.y*T.o.y+v.z*T.a.y+v.w*T.p.y;
        result.z=v.x*T.n.z+v.y*T.o.z+v.z*T.a.z+v.w*T.p.z;
        result.w=v.x*T.n.w+v.y*T.o.w+v.z*T.a.w+v.w*T.p.w;
 
        return(result);
}

h_transform assign_transform(T) /* assign one transform to another */
h_transform T;
{
        h_transform matrix;
 
        matrix.n.x=T.n.x;
        matrix.n.y=T.n.y;
        matrix.n.z=T.n.z;
        matrix.n.w=T.n.w;
          
        matrix.o.x=T.o.x;
        matrix.o.y=T.o.y;
        matrix.o.z=T.o.z;
        matrix.o.w=T.o.w;      
    
        matrix.a.x=T.a.x;
        matrix.a.y=T.a.y;
        matrix.a.z=T.a.z;
        matrix.a.w=T.a.w;
          
        matrix.p.x=T.p.x;
        matrix.p.y=T.p.y;
        matrix.p.z=T.p.z;
        matrix.p.w=T.p.w;
          
        return(matrix);
}

h_transform multTxT(T1,T2) /* multiply two transforms */
h_transform T1;
h_transform T2;
{
        h_transform result;
 
        result.n.x=T1.n.x*T2.n.x+T1.o.x*T2.n.y+T1.a.x*T2.n.z+T1.p.x*T2.n.w;
        result.n.y=T1.n.y*T2.n.x+T1.o.y*T2.n.y+T1.a.y*T2.n.z+T1.p.y*T2.n.w;
        result.n.z=T1.n.z*T2.n.x+T1.o.z*T2.n.y+T1.a.z*T2.n.z+T1.p.z*T2.n.w;
        result.n.w=T1.n.w*T2.n.x+T1.o.w*T2.n.y+T1.a.w*T2.n.z+T1.p.w*T2.n.w;
        
        result.o.x=T1.n.x*T2.o.x+T1.o.x*T2.o.y+T1.a.x*T2.o.z+T1.p.x*T2.o.w;
        result.o.y=T1.n.y*T2.o.x+T1.o.y*T2.o.y+T1.a.y*T2.o.z+T1.p.y*T2.o.w;
        result.o.z=T1.n.z*T2.o.x+T1.o.z*T2.o.y+T1.a.z*T2.o.z+T1.p.z*T2.o.w;
        result.o.w=T1.n.w*T2.o.x+T1.o.w*T2.o.y+T1.a.w*T2.o.z+T1.p.w*T2.o.w;
 
        result.a.x=T1.n.x*T2.a.x+T1.o.x*T2.a.y+T1.a.x*T2.a.z+T1.p.x*T2.a.w;
        result.a.y=T1.n.y*T2.a.x+T1.o.y*T2.a.y+T1.a.y*T2.a.z+T1.p.y*T2.a.w;
        result.a.z=T1.n.z*T2.a.x+T1.o.z*T2.a.y+T1.a.z*T2.a.z+T1.p.z*T2.a.w;
        result.a.w=T1.n.w*T2.a.x+T1.o.w*T2.a.y+T1.a.w*T2.a.z+T1.p.w*T2.a.w;
 
        result.p.x=T1.n.x*T2.p.x+T1.o.x*T2.p.y+T1.a.x*T2.p.z+T1.p.x*T2.p.w;
        result.p.y=T1.n.y*T2.p.x+T1.o.y*T2.p.y+T1.a.y*T2.p.z+T1.p.y*T2.p.w;
        result.p.z=T1.n.z*T2.p.x+T1.o.z*T2.p.y+T1.a.z*T2.p.z+T1.p.z*T2.p.w;
        result.p.w=T1.n.w*T2.p.x+T1.o.w*T2.p.y+T1.a.w*T2.p.z+T1.p.w*T2.p.w;
 
        return(result);
}
 
h_transform transform_inverse(T)  /* return inverse of a normalized transform */
h_transform T;
{
        h_transform inv;
 
	inv.n.x = T.n.x; inv.o.x = T.n.y; inv.a.x = T.n.z; inv.p.x = -dot(T.n,T.p);
	inv.n.y = T.o.x; inv.o.y = T.o.y; inv.a.y = T.o.z; inv.p.y = -dot(T.o,T.p);
	inv.n.z = T.a.x; inv.o.z = T.a.y; inv.a.z = T.a.z; inv.p.z = -dot(T.a,T.p);
	inv.n.w = 0;     inv.o.w = 0;     inv.a.w = 0;     inv.p.w = 1;
  
	return(inv);
}
