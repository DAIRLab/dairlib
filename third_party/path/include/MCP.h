#ifndef MCP_H
#define MCP_H

#include "Types.h"
#include "MCP_Interface.h"
#include "DenseVector.h"
#include "Evaluation.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* The header contains the declarations required for the MCP class.          */
/*                                                                           */
/* Currently, I am making modifications to the interface to make it easier   */
/* to hook up additional solvers.  The same template will be used for CNS.   */
/* I still need to work out the details on an MCP_Interface structure so that*/
/* you can pass in the interface routine functions.  There will then be      */
/* special routines for MCP_CNS which starts up a CNS and converts it to an  */
/* MCP automatically.  All of these will be put together into one library.   */
/*                                                                           */
/* The code will then look like:                                             */
/*     m = MCP_Start(mcp_interface);                                         */
/*     t = Algorithm(m);                                                     */
/*         MCP_Finish(m);                                                    */
/*****************************************************************************/

/*****************************************************************************/
/* Control functions                                                         */
/*                                                                           */
/* These function affect how the problem is generated.  They should only     */
/* be called before the main call to the algorithm (which calls the          */
/* initialization routines).                                                 */
/*                                                                           */
/* The reformulate option uses the following reformulation to remove the     */
/* equations/free variables from the problem.  We replace the original       */
/* system with:                                                              */
/*                                                                           */
/*          f(x, y - eta) perp   x >= 0                                      */
/*          g(x, y - eta) perp   y >= 0                                      */
/*   -e^T * g(x, y - eta) perp eta >= 0                                      */
/*                                                                           */
/* Where originally the y variables were free.  This does not appear to work */
/* in practice, but it also has not been extensively tested.                 */
/*                                                                           */
/* The preprocessing option currently is not completely implemented.  It     */
/* checks consistency of the bounds and removes fixed variables.             */
/*                                                                           */
/* The scaling option should check the jacobian and decide how to scale the  */
/* nonlinear model.  This has not been implemented.                          */
/*                                                                           */
/* The Jacobian parts have to do with speeding up the calculation of the     */
/* derivative.                                                               */
/*****************************************************************************/

Void MCP_Reformulate(MCP *m, Boolean b);
Void MCP_Scale(MCP *m, Boolean b);

/*****************************************************************************/
/* Manipulation functions.                                                   */
/*****************************************************************************/

Void MCP_Reference(MCP *m);
Void MCP_Clone(MCP *dest, MCP *source);

/*****************************************************************************/
/* Access methods.                                                           */
/*****************************************************************************/

Int MCP_GetAlgSize(MCP *m);	/* Actual size (only good after start)       */
Int MCP_GetAlgNNZ(MCP *m);     	/* Actual nnz estimate (only after start)    */
Int MCP_GetMaxAlgSize(MCP *m);	/* Maximum size seen by the algorithm        */
Int MCP_GetMaxAlgNNZ(MCP *m);   /* Maximum nnz as seen by the algorithm      */

DenseVector *MCP_GetAlgX(MCP *m);
DenseVector *MCP_GetAlgL(MCP *m);
DenseVector *MCP_GetAlgU(MCP *m);
IDenseVector *MCP_GetAlgT(MCP *m);

DenseVector *MCP_GetModF(MCP *m);
SparseMatrix *MCP_GetModJ(MCP *m);
Evaluation *MCP_GetModE(MCP *m);

DenseVector *MCP_GetModX(MCP *m);
DenseVector *MCP_GetModL(MCP *m);
DenseVector *MCP_GetModU(MCP *m);

MCP_Type MCP_GetProblemClass(MCP *m);

/*****************************************************************************/
/* These access methods are used to get basis information about the problem  */
/* under consideration.  Not all of the algorithms will use this information */
/* or fill in the data at the end of the call.  In these cases, the object   */
/* will make its best guess based upon the point passed in/out.              */
/*****************************************************************************/

IDenseVector *MCP_GetAlgBasis(MCP *m);
IDenseVector *MCP_GetModBasis(MCP *m);

/*****************************************************************************/
/* Modify methods.                                                           */
/*****************************************************************************/

Void MCP_SetProblemClass(MCP *m, MCP_Type t);

/*****************************************************************************/
/* Algorithm methods.                                                        */
/*****************************************************************************/

MCP_Termination MCP_Start(MCP *m);
Void MCP_Finish(MCP *m, MCP_Termination t,
		DenseVector *algX, IDenseVector *basX);

/*****************************************************************************/
/* Main functionality.                                                       */
/*                                                                           */
/* X and E should at least have size GetSize and GetNNZ                      */
/*****************************************************************************/

Boolean MCP_Function(MCP *m, DenseVector *algX, Evaluation *e);
Boolean MCP_Jacobian(MCP *m, DenseVector *algX, Evaluation *e);

Void MCP_VariableName(MCP *m, Int variable, Char *buffer, Int bufSize);
Void MCP_ConstraintName(MCP *m, Int constraint, Char *buffer, Int bufSize);

/*****************************************************************************/
/* Special functions for those problems defining an objective function.      */
/*****************************************************************************/

Boolean      MCP_Has_NLP(MCP *m);
Double       MCP_GetNLP_AlgF(MCP *m);
DenseVector *MCP_GetNLP_AlgG(MCP *m);
Double       MCP_GetNLP_ModF(MCP *m);
DenseVector *MCP_GetNLP_ModG(MCP *m);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef MCP_H */
