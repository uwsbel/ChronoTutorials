from __future__ import annotations
import typing
__all__: list[str] = ['ABSV', 'ABS_ACC', 'ACC', 'ANALYTICAL', 'ANG_ACC', 'ANG_VEL', 'BINARY', 'CENTERED_DIFFERENCE', 'CHPF', 'CHUNG', 'CLUMP', 'CNT_OUTPUT_CONTENT', 'CNT_TYPE', 'CNT_WILDCARD', 'COMPONENT', 'CSV', 'CUSTOM', 'ContactInfoContainer', 'DEBUG', 'DEMBoxGridSampler', 'DEMBoxHCPSampler', 'DEMClumpBatch', 'DEMClumpTemplate', 'DEMCylSurfSampler', 'DEMExternObj', 'DEMForceModel', 'DEMInitializer', 'DEMInspector', 'DEMMaterial', 'DEMMeshConnected', 'DEMSolver', 'DEMTrackedObj', 'DataContainer', 'ERROR', 'EXP_FACTOR', 'EXTENDED_TAYLOR', 'FAMILY', 'FORCE', 'FORCE_MODEL', 'FORWARD_EULER', 'FrameTransformGlobalToLocal', 'FrameTransformLocalToGlobal', 'GEO_ID', 'GEO_WILDCARD', 'GetDEMEDataFile', 'GridSampler', 'HCPSampler', 'HERTZIAN', 'HERTZIAN_FRICTIONLES', 'INFO', 'MAT', 'MESH', 'MESH_FORMAT', 'NICKNAME', 'NONE', 'NORMAL', 'OBJ', 'OUTPUT_CONTENT', 'OUTPUT_FORMAT', 'OWNER', 'OWNER_TYPE', 'OWNER_WILDCARD', 'PDSampler', 'PI', 'POINT', 'QUAT', 'QUIET', 'RuntimeDataHelper', 'SPATIAL_DIR', 'STEP_ANOMALY', 'STEP_DEBUG', 'STEP_METRIC', 'TIME_INTEGRATOR', 'TORQUE', 'Tracker', 'VEL', 'VERBOSITY', 'VTK', 'WARNING', 'X', 'XYZ', 'Y', 'Z']
class CNT_OUTPUT_CONTENT:
    """
    Members:
    
      CNT_TYPE
    
      FORCE
    
      POINT
    
      COMPONENT
    
      NORMAL
    
      TORQUE
    
      CNT_WILDCARD
    
      OWNER
    
      GEO_ID
    
      NICKNAME
    """
    CNT_TYPE: typing.ClassVar[CNT_OUTPUT_CONTENT]  # value = <CNT_OUTPUT_CONTENT.CNT_TYPE: 0>
    CNT_WILDCARD: typing.ClassVar[CNT_OUTPUT_CONTENT]  # value = <CNT_OUTPUT_CONTENT.CNT_WILDCARD: 32>
    COMPONENT: typing.ClassVar[CNT_OUTPUT_CONTENT]  # value = <CNT_OUTPUT_CONTENT.COMPONENT: 4>
    FORCE: typing.ClassVar[CNT_OUTPUT_CONTENT]  # value = <CNT_OUTPUT_CONTENT.FORCE: 1>
    GEO_ID: typing.ClassVar[CNT_OUTPUT_CONTENT]  # value = <CNT_OUTPUT_CONTENT.GEO_ID: 128>
    NICKNAME: typing.ClassVar[CNT_OUTPUT_CONTENT]  # value = <CNT_OUTPUT_CONTENT.NICKNAME: 256>
    NORMAL: typing.ClassVar[CNT_OUTPUT_CONTENT]  # value = <CNT_OUTPUT_CONTENT.NORMAL: 8>
    OWNER: typing.ClassVar[CNT_OUTPUT_CONTENT]  # value = <CNT_OUTPUT_CONTENT.OWNER: 64>
    POINT: typing.ClassVar[CNT_OUTPUT_CONTENT]  # value = <CNT_OUTPUT_CONTENT.POINT: 2>
    TORQUE: typing.ClassVar[CNT_OUTPUT_CONTENT]  # value = <CNT_OUTPUT_CONTENT.TORQUE: 16>
    __members__: typing.ClassVar[dict[str, CNT_OUTPUT_CONTENT]]  # value = {'CNT_TYPE': <CNT_OUTPUT_CONTENT.CNT_TYPE: 0>, 'FORCE': <CNT_OUTPUT_CONTENT.FORCE: 1>, 'POINT': <CNT_OUTPUT_CONTENT.POINT: 2>, 'COMPONENT': <CNT_OUTPUT_CONTENT.COMPONENT: 4>, 'NORMAL': <CNT_OUTPUT_CONTENT.NORMAL: 8>, 'TORQUE': <CNT_OUTPUT_CONTENT.TORQUE: 16>, 'CNT_WILDCARD': <CNT_OUTPUT_CONTENT.CNT_WILDCARD: 32>, 'OWNER': <CNT_OUTPUT_CONTENT.OWNER: 64>, 'GEO_ID': <CNT_OUTPUT_CONTENT.GEO_ID: 128>, 'NICKNAME': <CNT_OUTPUT_CONTENT.NICKNAME: 256>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class ContactInfoContainer(DataContainer):
    def GetAGeo(self) -> list[int]:
        """
        Get AGeo number from the container.
        """
    def GetAOwner(self) -> list[int]:
        """
        Get AOwner number from the container.
        """
    def GetAOwnerFamily(self) -> list[int]:
        """
        Get AOwnerFamily number from the container.
        """
    def GetBGeo(self) -> list[int]:
        """
        Get BGeo number from the container.
        """
    def GetBOwner(self) -> list[int]:
        """
        Get BOwner number from the container.
        """
    def GetBOwnerFamily(self) -> list[int]:
        """
        Get BOwnerFamily number from the container.
        """
    def GetContactType(self) -> list[str]:
        """
        Get contact type as strings from the container.
        """
    def GetForce(self) -> list[float3]:
        """
        Get force as vectors from the container.
        """
    def GetNormal(self) -> list[float3]:
        """
        Get contact normal as vectors from the container.
        """
    def GetPoint(self) -> list[float3]:
        """
        Get contact points as vectors from the container.
        """
    def GetTorque(self) -> list[float3]:
        """
        Get torque as vectors from the container.
        """
    def __init__(self, arg0: int, arg1: list[tuple[str, str]]) -> None:
        ...
class DEMClumpBatch(DEMInitializer):
    def AddExistingContactWildcard(self, arg0: str, arg1: list[float]) -> None:
        ...
    @typing.overload
    def AddGeometryWildcard(self, arg0: str, arg1: list[float]) -> None:
        ...
    @typing.overload
    def AddGeometryWildcard(self, arg0: str, arg1: float) -> None:
        ...
    @typing.overload
    def AddOwnerWildcard(self, arg0: str, arg1: list[float]) -> None:
        ...
    @typing.overload
    def AddOwnerWildcard(self, arg0: str, arg1: float) -> None:
        ...
    def GetNumClumps(self) -> int:
        ...
    def GetNumContacts(self) -> int:
        ...
    def GetNumSpheres(self) -> int:
        ...
    def SetExistingContactWildcards(self, arg0: dict[str, list[float]]) -> None:
        ...
    def SetExistingContacts(self, arg0: list[tuple[int, int]]) -> None:
        ...
    @typing.overload
    def SetFamilies(self, arg0: list[int]) -> None:
        ...
    @typing.overload
    def SetFamilies(self, arg0: int) -> None:
        ...
    def SetFamily(self, arg0: int) -> None:
        ...
    def SetGeometryWildcards(self, arg0: dict[str, list[float]]) -> None:
        ...
    def SetOwnerWildcards(self, arg0: dict[str, list[float]]) -> None:
        ...
    def SetType(self, arg0: DEMClumpTemplate) -> None:
        ...
    @typing.overload
    def SetTypes(self, arg0: list[DEMClumpTemplate]) -> None:
        ...
    @typing.overload
    def SetTypes(self, arg0: DEMClumpTemplate) -> None:
        ...
    @typing.overload
    def SetVel(self, arg0: list[list[float]]) -> None:
        ...
    @typing.overload
    def SetVel(self, arg0: list[float]) -> None:
        ...
    def __init__(self, arg0: int) -> None:
        ...
class DEMClumpTemplate:
    def AssignName(self, arg0: str) -> None:
        ...
    def InformCentroidPrincipal(self, arg0: list[float], arg1: list[float]) -> None:
        ...
    def MOI(self) -> list[float]:
        ...
    def Mass(self) -> float:
        ...
    def Move(self, arg0: list[float], arg1: list[float]) -> None:
        ...
    def ReadComponentFromFile(self, filename: str, x_id: str = 'x', y_id: str = 'y', z_id: str = 'z', r_id: str = 'r') -> int:
        """
        Retrieve clump's sphere component information from a file
        """
    def Scale(self, arg0: float) -> None:
        ...
    def SetMOI(self, arg0: list[float]) -> None:
        ...
    def SetMass(self, arg0: float) -> None:
        ...
    @typing.overload
    def SetMaterial(self, arg0: list[DEMMaterial]) -> None:
        ...
    @typing.overload
    def SetMaterial(self, arg0: DEMMaterial) -> None:
        ...
    def SetVolume(self, arg0: float) -> None:
        ...
    def __init__(self) -> None:
        ...
class DEMExternObj(DEMInitializer):
    MOI: float3
    entity_params: list[...]
    family_code: int
    init_oriQ: float4
    init_pos: float3
    load_order: int
    mass: float
    materials: list[DEMMaterial]
    types: list[...]
    def AddCylinder(self, pos: list[float], axis: list[float], rad: float, material: DEMMaterial, normal: bool = False) -> None:
        """
        Add a cylinder of infinite length, which is along a user-specific axis
        """
    def AddPlane(self, arg0: list[float], arg1: list[float], arg2: DEMMaterial) -> None:
        """
        Add a plane with infinite size.
        """
    def AddZCylinder(self, pos: list[float], rad: float, material: DEMMaterial, normal: bool = False) -> None:
        """
        Add a z-axis-aligned cylinder of infinite length
        """
    def Mass(self) -> float:
        ...
    def SetFamily(self, arg0: int) -> None:
        """
        Defines an object contact family number
        """
    def SetInitPos(self, arg0: list[float]) -> None:
        """
        Set the initial position for this object (before simulation initializes).
        """
    def SetInitQuat(self, arg0: list[float]) -> None:
        """
        Set the initial quaternion for this object (before simulation initializes).
        """
    def SetMOI(self, arg0: list[float]) -> None:
        """
        Sets the MOI (in the principal frame)
        """
    def SetMass(self, arg0: float) -> None:
        """
        Sets the mass of this object
        """
    def __init__(self) -> None:
        ...
class DEMForceModel:
    def DefineCustomModel(self, arg0: str) -> None:
        """
        Define user-custom force model with a string which is your force calculation code.
        """
    def DefineCustomModelPrerequisites(self, arg0: str) -> None:
        """
        Define user-custom force model's utility __device__ functions with a string.
        """
    def ReadCustomModelFile(self, arg0: ...) -> int:
        """
        Read user-custom force model from a file (which by default should reside in kernel/DEMUserScripts), which contains your force calculation code. Returns 0 if read successfully, otherwise 1.
        """
    def ReadCustomModelPrerequisitesFile(self, arg0: ...) -> int:
        """
        Read user-custom force model's utility __device__ functions from a file (which by default should reside in kernel/DEMUserScripts). Returns 0 if read successfully, otherwise 1.
        """
    def SetForceModelType(self, arg0: ...) -> None:
        """
        Set the contact force model type
        """
    def SetMustHaveMatProp(self, arg0: set[str]) -> None:
        """
        Specifiy the material properties that this force model will use
        """
    def SetMustPairwiseMatProp(self, arg0: set[str]) -> None:
        """
        Specifiy the material properties that are pair-wise (instead of being associated with each individual material).
        """
    def SetPerContactWildcards(self, arg0: set[str]) -> None:
        """
        Set the names for the extra quantities that will be associated with each contact pair. For example, history-based models should have 3 float arrays to store contact history. Only float is supported. Note the initial value of all contact wildcard arrays is automatically 0
        """
    def SetPerGeometryWildcards(self, arg0: set[str]) -> None:
        """
        Set the names for the extra quantities that will be associated with each geometry. For example, you can use this to associate certain electric charges to each particle's each component which represents a distribution of the charges. Only float is supported.
        """
    def SetPerOwnerWildcards(self, arg0: set[str]) -> None:
        """
         Set the names for the extra quantities that will be associated with each owner. For example, you can use this to associate a cohesion parameter to each particle. Only float is supported.
        """
    def __init__(self, arg0: ...) -> None:
        ...
class DEMInitializer:
    def __init__(self) -> None:
        ...
class DEMInspector:
    def GetValue(self) -> float:
        ...
    def __init__(self, arg0: ..., arg1: ..., arg2: str) -> None:
        ...
class DEMMaterial:
    load_order: int
    mat_prop: dict[str, float]
    def __init__(self, arg0: dict[str, float]) -> None:
        ...
class DEMMeshConnected(DEMInitializer):
    @typing.overload
    def AddGeometryWildcard(self, arg0: str, arg1: list[float]) -> None:
        ...
    @typing.overload
    def AddGeometryWildcard(self, arg0: str, arg1: float) -> None:
        ...
    def Clear(self) -> None:
        """
        Clears everything from memory
        """
    def ClearWildcards(self) -> None:
        ...
    def GetCoordsVertices(self) -> list[list[float]]:
        ...
    def GetIndicesVertexes(self) -> list[list[int]]:
        ...
    def GetNumNodes(self) -> int:
        """
        Get the number of nodes in the mesh
        """
    def GetNumTriangles(self) -> int:
        """
        Get the number of triangles already added to this mesh
        """
    def GetTriangle(self, arg0: int) -> ...:
        """
        Access the n-th triangle in mesh
        """
    def InformCentroidPrincipal(self, arg0: list[float], arg1: list[float]) -> None:
        ...
    def LoadWavefrontMesh(self, input_file: str, load_normals: bool = True, load_uv: bool = False) -> bool:
        """
        Load a triangle mesh saved as a Wavefront .obj file
        """
    def MOI(self) -> list[float]:
        ...
    def Mass(self) -> float:
        ...
    def Mirror(self, arg0: list[float], arg1: list[float]) -> None:
        ...
    def Move(self, arg0: list[float], arg1: list[float]) -> None:
        ...
    @typing.overload
    def Scale(self, arg0: float) -> None:
        ...
    @typing.overload
    def Scale(self, arg0: list[float]) -> None:
        ...
    def SetFamily(self, arg0: int) -> None:
        ...
    def SetGeometryWildcards(self, arg0: dict[str, list[float]]) -> None:
        ...
    def SetInitPos(self, arg0: list[float]) -> None:
        ...
    def SetInitQuat(self, arg0: list[float]) -> None:
        ...
    def SetMOI(self, arg0: list[float]) -> None:
        ...
    def SetMass(self, arg0: float) -> None:
        ...
    @typing.overload
    def SetMaterial(self, arg0: DEMMaterial) -> None:
        ...
    @typing.overload
    def SetMaterial(self, arg0: list[DEMMaterial]) -> None:
        ...
    def UseNormals(self, use: bool = True) -> None:
        """
        Instruct that when the mesh is initialized into the system, it will re-order the nodes of each triangle so that the normals derived from right-hand-rule are the same as the normals in the mesh file
        """
    def WriteWavefront(self: str, arg0: list[DEMMeshConnected]) -> None:
        """
        Write the specified meshes in a Wavefront .obj file
        """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: str) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: str, arg1: DEMMaterial) -> None:
        ...
class DEMSolver:
    def AddBCPlane(self, arg0: list[float], arg1: list[float], arg2: ...) -> ...:
        """
        Add an analytical plane to the simulation.
        """
    @typing.overload
    def AddClumps(self, arg0: ...) -> DEMClumpBatch:
        """
        Load input clumps (topology types and initial locations) on a per-pair basis. Note that the initial location means the location of the clumps' CoM coordinates in the global frame.
        """
    @typing.overload
    def AddClumps(self, arg0: ..., arg1: list[list[float]]) -> DEMClumpBatch:
        """
        Load input clumps (topology types and initial locations) on a per-pair basis. Note that the initial location means the location of the clumps' CoM coordinates in the global frame.
        """
    @typing.overload
    def AddClumps(self, arg0: list[...], arg1: list[list[float]]) -> DEMClumpBatch:
        """
        Load input clumps (topology types and initial locations) on a per-pair basis. Note that the initial location means the location of the clumps' CoM coordinates in the global frame.
        """
    def AddExternalObject(self) -> ...:
        """
        Add an analytical object to the simulation system.
        """
    def AddFamilyPrescribedAcc(self, ID: int, X: str, Y: str, Z: str, pre: str = 'none') -> None:
        """
        The entities in this family will always experienced an extra acceleration defined using this method.
        """
    def AddFamilyPrescribedAngAcc(self, ID: int, X: str, Y: str, Z: str, pre: str = 'none') -> None:
        """
        The entities in this family will always experienced an extra angular acceleration defined using this method.
        """
    def AddKernelInclude(self, arg0: str) -> None:
        """
        Add a library that the kernels will be compiled with (so that the user can use the provided methods in their customized code, like force model).
        """
    def AddOwnerNextStepAcc(self, arg0: int, arg1: list[float3]) -> None:
        """
        Add an extra acceleration to a owner for the next time step.
        """
    def AddOwnerNextStepAngAcc(self, arg0: int, arg1: list[float3]) -> None:
        """
         Add an extra angular acceleration to a owner for the next time step.
        """
    @typing.overload
    def AddWavefrontMeshObject(self, filename: str, mat: ..., load_normals: bool = True, load_uv: bool = False) -> DEMMeshConnected:
        """
        Load a mesh-represented object.
        """
    @typing.overload
    def AddWavefrontMeshObject(self, arg0: ...) -> DEMMeshConnected:
        """
        Load a mesh-represented object.
        """
    @typing.overload
    def AddWavefrontMeshObject(self, filename: str, load_normals: bool = True, load_uv: bool = False) -> DEMMeshConnected:
        """
        Load a mesh-represented object.
        """
    def ChangeClumpFamily(self, fam_num: int, X: tuple[float, float] = (-1000000000000000.0, 1000000000000000.0), Y: tuple[float, float] = (-1000000000000000.0, 1000000000000000.0), Z: tuple[float, float] = (-1000000000000000.0, 1000000000000000.0), orig_fam: set[int] = set()) -> int:
        """
        Change the family number for the clumps in a box region to the specified value.
        """
    def ChangeFamily(self, arg0: int, arg1: int) -> None:
        """
        Change all entities with family number ID_from to have a new number ID_to, when the condition defined by the string is satisfied by the entities in question. This should be called before initialization, and will be baked into the solver, so the conditions will be checked and changes applied every time step.
        """
    def ChangeFamilyWhen(self, arg0: int, arg1: int, arg2: str) -> None:
        """
        Change all entities with family number ID_from to have a new number ID_to, when the condition defined by the string is satisfied by the entities in question. This should be called before initialization, and will be baked into the solver, so the conditions will be checked and changes applied every time step.
        """
    def ClearCache(self) -> None:
        """
        Remove host-side cached vectors (so you can re-define them, and then re-initialize system).
        """
    def ClearThreadCollaborationStats(self) -> None:
        """
        Reset the collaboration stats between dT and kT back to the initial value (0). You should call this if you want to start over and re-inspect the stats of the new run; otherwise, it is generally not needed, you can go ahead and destroy DEMSolver.
        """
    def ClearTimingStats(self) -> None:
        """
        Reset the recordings of the wall time and percentages of wall time spend on various solver tasks.
        """
    def CorrectFamilyAngVel(self, ID: int, X: str, Y: str, Z: str, pre: str = 'none') -> None:
        """
        The entities in this family will always experience an added angular-velocity correction defined using this method. At the same time, they are still subject to the `simulation physics'.
        """
    def CorrectFamilyLinVel(self, ID: int, X: str, Y: str, Z: str, pre: str = 'none') -> None:
        """
        The entities in this family will always experience an added linear-velocity correction defined using this method. At the same time, they are still subject to the `simulation physics'.
        """
    def CorrectFamilyPosition(self, ID: int, X: str, Y: str, Z: str, pre: str = 'none') -> None:
        """
        The entities in this family will always experience an added positional correction defined using this method. At the same time, they are still subject to the `simulation physics'.
        """
    def CorrectFamilyQuaternion(self, ID: int, q_formula: str) -> None:
        """
        The entities in this family will always experience an added quaternion correction defined using this method. At the same time, they are still subject to the `simulation physics'.
        """
    @typing.overload
    def CreateInspector(self, quantity: str = 'clump_max_z') -> DEMInspector:
        """
        Create a inspector object that can help query some statistical info of the clumps in the simulation.
        """
    @typing.overload
    def CreateInspector(self, arg0: str, arg1: str) -> DEMInspector:
        """
        Create a inspector object that can help query some statistical info of the clumps in the simulation.
        """
    def DefineContactForceModel(self, arg0: str) -> DEMForceModel:
        """
        Define a custom contact force model by a string. Returns a pointer to the force model in use.
        """
    def DisableAdaptiveBinSize(self) -> None:
        """
        Disable the use of adaptive bin size (always use initial size)
        """
    def DisableAdaptiveUpdateFreq(self) -> None:
        """
        Disable the use of adaptive max update step count (always use initial update frequency)
        """
    def DisableContactBetweenFamilies(self, arg0: int, arg1: int) -> None:
        """
        Instruct the solver that the 2 input families should not have contacts (a.k.a. ignored, if such a pair is encountered in contact detection). These 2 families can be the same (which means no contact within members of that family).
        """
    def DisableFamilyOutput(self, arg0: int) -> None:
        """
        Prevent entites associated with this family to be outputted to files.
        """
    def DoDynamics(self, arg0: float) -> None:
        """
        Advance simulation by this amount of time (but does not attempt to sync kT and dT). This can work with both long and short call durations and allows interplay with co-simulation APIs.
        """
    def DoDynamicsThenSync(self, arg0: float) -> None:
        """
        Advance simulation by this amount of time, and at the end of this call, synchronize kT and dT. This is suitable for a longer call duration and without co-simulation.
        """
    def DoStepDynamics(self) -> None:
        """
        Equivalent to calling DoDynamics with the time step size as the argument.
        """
    @typing.overload
    def Duplicate(self, arg0: ...) -> ...:
        """
        Duplicate a material that is loaded into the system.
        """
    @typing.overload
    def Duplicate(self, arg0: ...) -> ...:
        """
        Duplicate a clump template that is loaded into the system.
        """
    @typing.overload
    def Duplicate(self, arg0: ...) -> ...:
        """
        Duplicate a batch of clumps that is loaded into the system.
        """
    def EnableContactBetweenFamilies(self, arg0: int, arg1: int) -> None:
        """
        Re-enable contact between 2 families after the system is initialized.
        """
    def EnsureKernelErrMsgLineNum(self, flag: bool = True) -> None:
        """
        If true, each jitification string substitution will do a one-liner to one-liner replacement, so that if the kernel compilation fails, the error meessage line number will reflex the actual spot where that happens (instead of some random number).
        """
    def GetAllOwnerWildcardValue(self, arg0: str) -> list[float]:
        """
        Get the owner wildcard's values of all entities.
        """
    def GetAnalWildcardValue(self, arg0: int, arg1: str, arg2: int) -> list[float]:
        """
        Get the geometry wildcard's values of a series of analytical entities.
        """
    def GetAvgSphContacts(self) -> float:
        """
        Get the current number of contacts each sphere has
        """
    def GetBinNum(self) -> int:
        """
        Get the current number of bins (for contact detection). Must be called from synchronized stance.
        """
    def GetBinSize(self) -> float:
        """
        Get the current bin (for contact detection) size. Must be called from synchronized stance.
        """
    def GetCachedMesh(self, arg0: int) -> ...:
        """
        Get a handle for the mesh this tracker is tracking.
        """
    @typing.overload
    def GetClumpContacts(self) -> list[tuple[int, int]]:
        """
        Get all clump--clump contact ID pairs in the simulation system. Note all GetContact-like methods reports potential contacts (not necessarily confirmed contacts), meaning they are similar to what WriteContactFileIncludingPotentialPairs does, not what WriteContactFile does.
        """
    @typing.overload
    def GetClumpContacts(self, arg0: set[int]) -> list[tuple[int, int]]:
        """
        Get all clump--clump contact ID pairs in the simulation system. Note all GetContact-like methods reports potential contacts (not necessarily confirmed contacts), meaning they are similar to what WriteContactFileIncludingPotentialPairs does, not what WriteContactFile does. Use the argument to include only these families in the output.
        """
    @typing.overload
    def GetClumpContacts(self, arg0: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """
        Get all clump--clump contact ID pairs in the simulation system. Note all GetContact-like methods reports potential contacts (not necessarily confirmed contacts), meaning they are similar to what WriteContactFileIncludingPotentialPairs does, not what WriteContactFile does.
        """
    def GetContactDetailedInfo(self, force_thres: float = -1.0) -> ContactInfoContainer:
        """
        Get all contact pairs' detailed information (actual content based on the setting with SetContactOutputContent; default are owner IDs, contact point location, contact force, and associated wildcard values) in the simulation system. Note all GetContact-like methods reports potential contacts (not necessarily confirmed contacts), meaning they are similar to what WriteContactFileIncludingPotentialPairs does, not what WriteContactFile does.
        """
    def GetContactForceModel(self) -> DEMForceModel:
        """
        Get the current force model
        """
    @typing.overload
    def GetContacts(self) -> list[tuple[int, int]]:
        """
        Get all contact ID pairs in the simulation system. Note all GetContact-like methods reports potential contacts (not necessarily confirmed contacts), meaning they are similar to what WriteContactFileIncludingPotentialPairs does, not what WriteContactFile does.
        """
    @typing.overload
    def GetContacts(self, arg0: set[int]) -> list[tuple[int, int]]:
        """
        Get all contact ID pairs in the simulation system. Note all GetContact-like methods reports potential contacts (not necessarily confirmed contacts), meaning they are similar to what WriteContactFileIncludingPotentialPairs does, not what WriteContactFile does. Use the argument to include only these families in the output.
        """
    @typing.overload
    def GetContacts(self, arg0: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """
        Get all contact ID pairs in the simulation system. Note all GetContact-like methods reports potential contacts (not necessarily confirmed contacts), meaning they are similar to what WriteContactFileIncludingPotentialPairs does, not what WriteContactFile does.
        """
    def GetDeviceMemUsageDynamic(self) -> int:
        """
        Get the device memory usage (in bytes) on dT.
        """
    def GetDeviceMemUsageKinematic(self) -> int:
        """
        Get the device memory usage (in bytes) on kT.
        """
    def GetFamilyOwnerWildcardValue(self, arg0: int, arg1: str) -> list[float]:
        """
        Get the owner wildcard's values of all entities in family N.
        """
    def GetHostMemUsageDynamic(self) -> int:
        """
        Get the host memory usage (in bytes) on dT.
        """
    def GetHostMemUsageKinematic(self) -> int:
        """
        Get the host memory usage (in bytes) on kT.
        """
    def GetInitStatus(self) -> bool:
        """
        Return whether this simulation system is initialized.
        """
    def GetJitStringSubs(self) -> dict[str, str]:
        """
        Get the jitification string substitution laundary list. It is needed by some of this simulation system's friend classes.
        """
    def GetJitifyOptions(self) -> list[str]:
        """
        Get current jitification options. It is needed by some of this simulation system's friend classes.
        """
    def GetMeshNodesGlobal(self, arg0: int) -> list[float3]:
        """
        Get the current locations of all the nodes in the mesh being tracked.
        """
    def GetNumClumps(self) -> int:
        """
        Return the number of clumps that are currently in the simulation. Must be used after initialization.
        """
    def GetNumContacts(self) -> int:
        """
        Get the number of kT-reported potential contact pairs.
        """
    def GetNumOwners(self) -> int:
        """
        Return the total number of owners (clumps + meshes + analytical objects) that are currently in the simulation. Must be used after initialization.
        """
    def GetOwnerAcc(self, ownerID: int, n: int = 1) -> list[float3]:
        """
        Get the acceleration of n consecutive owners.
        """
    def GetOwnerAngAcc(self, ownerID: int, n: int = 1) -> list[float3]:
        """
        Get the angular acceleration of n consecutive owners.
        """
    def GetOwnerAngVel(self, ownerID: int, n: int = 1) -> list[float3]:
        """
        Get angular velocity of n consecutive owners.
        """
    def GetOwnerContactClumps(self, arg0: int) -> list[int]:
        """
        Get the clumps that are in contact with this owner as a vector.
        """
    @typing.overload
    def GetOwnerContactForces(self, arg0: list[int], arg1: list[float3], arg2: list[float3]) -> int:
        """
        Get all contact forces that concern a list of owners.
        """
    @typing.overload
    def GetOwnerContactForces(self, ownerIDs: list[int], points: list[float3], forces: list[float3], poitorquesnts: list[float3], torque_in_local: bool = False) -> int:
        """
        Get all contact forces and torque that concern a list of owners.
        """
    def GetOwnerFamily(self, ownerID: int, n: int = 1) -> list[int]:
        """
        Get the family number of n consecutive owners.
        """
    def GetOwnerMOI(self, ownerID: int, n: int = 1) -> list[float3]:
        """
        Get the moment of inertia (in principal axis frame) of n consecutive owners.
        """
    def GetOwnerMass(self, ownerID: int, n: int = 1) -> list[float]:
        """
        Get the mass of n consecutive owners.
        """
    def GetOwnerOriQ(self, ownerID: int, n: int = 1) -> list[float4]:
        """
        Get quaternion of n consecutive owners.
        """
    def GetOwnerPosition(self, ownerID: int, n: int = 1) -> list[float3]:
        """
        Get position of n consecutive owners.
        """
    def GetOwnerVelocity(self, ownerID: int, n: int = 1) -> list[float3]:
        """
        Get velocity of n consecutive owners.
        """
    def GetOwnerWildcardValue(self, ownerID: int, name: str, n: int = 1) -> list[float]:
        """
        Get the owner wildcard's values of some owners.
        """
    def GetSimTime(self) -> float:
        """
        Get the simulation time passed since the start of simulation.
        """
    def GetSphereWildcardValue(self, arg0: int, arg1: str, arg2: int) -> list[float]:
        """
        Get the geometry wildcard's values of a series of spheres.
        """
    def GetTimeStepSize(self) -> float:
        """
        Get the current time step size in simulation.
        """
    def GetTriWildcardValue(self, arg0: int, arg1: str, arg2: int) -> list[float]:
        """
        Get the geometry wildcard's values of a series of triangles.
        """
    def GetUpdateFreq(self) -> float:
        """
        Get the current update frequency used by the solver.
        """
    def GetWhetherForceCollectInKernel(self) -> bool:
        """
        Return whether the solver is currently reducing force in the force calculation kernel.
        """
    def Initialize(self, dry_run: bool = False) -> None:
        """
        Initializes the system.
        """
    def InstructBoxDomainBoundingBC(self, arg0: str, arg1: ...) -> None:
        """
        Instruct if and how we should add boundaries to the simulation world upon initialization. Choose between `none', `all' (add 6 boundary planes) and `top_open' (add 5 boundary planes and leave the z-directon top open). Also specifies the material that should be assigned to those bounding boundaries.
        """
    @typing.overload
    def InstructBoxDomainDimension(self, x: float, y: float, z: float, dir_exact: str = 'none') -> None:
        """
        Set the Box Domain Dimension
        """
    @typing.overload
    def InstructBoxDomainDimension(self, x: tuple[float, float], y: tuple[float, float], z: tuple[float, float], dir_exact: str = 'none') -> None:
        """
        Set the span of the Box Domain
        """
    def InstructNumOwners(self, arg0: int) -> None:
        """
        Explicitly instruct the sizes for the arrays at initialization time. This is useful when the number of owners tends to change (especially gradually increase) frequently in the simulation.
        """
    @typing.overload
    def LoadClumpType(self, arg0: float, arg1: list[float], arg2: str, arg3: ...) -> ...:
        """
        Load a clump type into the API-level cache
        """
    @typing.overload
    def LoadClumpType(self, arg0: float, arg1: list[float], arg2: list[float], arg3: list[list[float]], arg4: ...) -> ...:
        """
        Load a clump type into the API-level cache
        """
    @typing.overload
    def LoadClumpType(self, arg0: ...) -> ...:
        """
        Load a clump type into the API-level cache
        """
    @typing.overload
    def LoadClumpType(self, arg0: float, arg1: list[float], arg2: str, arg3: list[...]) -> ...:
        """
        Load a clump type into the API-level cache
        """
    @typing.overload
    def LoadClumpType(self, arg0: float, arg1: list[float], arg2: str, arg3: ...) -> ...:
        """
        Load a clump type into the API-level cache
        """
    @typing.overload
    def LoadMaterial(self, arg0: dict[str, float]) -> ...:
        """
        Load materials properties (Young's modulus, Poisson's ratio...) into the system.
        """
    @typing.overload
    def LoadMaterial(self, arg0: ...) -> ...:
        """
        Load materials properties into the system.
        """
    def LoadSphereType(self, arg0: float, arg1: float, arg2: ...) -> ...:
        ...
    def MarkFamilyPersistentContact(self, arg0: int, arg1: int) -> None:
        """
        Make it so that if for any currently-existing contact, if its two contact geometries are in family N1 and N2 respectively, this contact will never be removed.
        """
    def MarkFamilyPersistentContactBoth(self, arg0: int) -> None:
        """
        Make it so that for any currently-existing contact, if both of its contact geometries are in family N, then this contact will never be removed.
        """
    def MarkFamilyPersistentContactEither(self, arg0: int) -> None:
        """
        Make it so that for any currently-existing contact, if one of its contact geometries is in family N, then this contact will never be removed.
        """
    def MarkPersistentContact(self) -> None:
        """
        Make it so that all currently-existing contacts in this simulation will never be removed.
        """
    def PrintKinematicScratchSpaceUsage(self) -> None:
        """
        Print kT's scratch space usage. This is a debug method.
        """
    def PurgeFamily(self, arg0: int) -> None:
        ...
    def ReadContactForceModel(self, arg0: str) -> DEMForceModel:
        """
        Read user custom contact force model from a file (which by default should reside in kernel/DEMUserScripts). Returns a pointer to the force model in use.
        """
    def ReleaseFlattenedArrays(self) -> None:
        ...
    def RemoveFamilyPersistentContact(self, arg0: int, arg1: int) -> None:
        """
        Cancel contact persistence qualification. Work like the inverse of MarkFamilyPersistentContact.
        """
    def RemoveFamilyPersistentContactBoth(self, arg0: int) -> None:
        """
        Cancel contact persistence qualification. Work like the inverse of MarkFamilyPersistentContactBoth.
        """
    def RemoveFamilyPersistentContactEither(self, arg0: int) -> None:
        """
        Cancel contact persistence qualification. Work like the inverse of MarkFamilyPersistentContactEither.
        """
    def RemoveKernelInclude(self) -> None:
        """
        Remove all extra libraries that the kernels `include' in their headers.
        """
    def RemovePersistentContact(self) -> None:
        """
        Cancel contact persistence qualification. Work like the inverse of MarkPersistentContact.
        """
    def SetAdaptiveBinSizeAcc(self, arg0: float) -> None:
        """
        Set how fast kT changes the direction of bin size adjustmemt when there's a more beneficial direction
        """
    def SetAdaptiveBinSizeDelaySteps(self, arg0: int) -> None:
        """
        Adjust how frequent kT updates the bin size
        """
    def SetAdaptiveBinSizeLowerProactivity(self, arg0: float) -> None:
        """
        Set how proactive the solver is in avoiding the bin being too small (leading to too many bins in domain).
        """
    def SetAdaptiveBinSizeMaxRate(self, arg0: float) -> None:
        """
        Set the max rate that the bin size can change in one adjustment
        """
    def SetAdaptiveBinSizeUpperProactivity(self, arg0: float) -> None:
        """
        Set how proactive the solver is in avoiding the bin being too big (leading to too many geometries in a bin)
        """
    def SetAdaptiveTimeStepType(self, arg0: str) -> None:
        """
        Set the strategy for auto-adapting time step size (NOT implemented, no effect yet).
        """
    def SetAnalWildcardValue(self, arg0: int, arg1: str, arg2: list[float]) -> None:
        """
        Set the wildcard values of some analytical components.
        """
    def SetCDMaxUpdateFreq(self, arg0: int) -> None:
        """
        Set the upper bound of kT update frequency (when it is adjusted automatically).
        """
    def SetCDNumStepsMaxDriftAheadOfAvg(self, arg0: float) -> None:
        """
        Set the number of steps dT configures its max drift more than average drift steps.
        """
    def SetCDNumStepsMaxDriftHistorySize(self, arg0: int) -> None:
        ...
    def SetCDNumStepsMaxDriftMultipleOfAvg(self, arg0: float) -> None:
        """
        Set the multiplier which dT configures its max drift to be w.r.t. the average drift steps.
        """
    def SetCDUpdateFreq(self, arg0: int) -> None:
        """
        Set the number of dT steps before it waits for a contact-pair info update from kT.
        """
    def SetCollectAccRightAfterForceCalc(self, flag: bool = True) -> None:
        """
        Reduce contact forces to accelerations right after calculating them, in the same kernel. This may give some performance boost if you have only polydisperse spheres, no clumps.
        """
    def SetContactOutputContent(self, arg0: list[str]) -> None:
        """
        Specify the information that needs to go into the contact pair output files.
        """
    def SetContactOutputFormat(self, arg0: str) -> None:
        """
        Specify the file format of contact pairs.
        """
    def SetContactWildcardValue(self, arg0: str, arg1: float) -> None:
        """
        Change the value of contact wildcards to val. Apply to all simulation bodies that are present.
        """
    def SetContactWildcards(self, arg0: set[str]) -> None:
        """
        Set the names for the extra quantities that will be associated with each contact pair.
        """
    def SetErrorOutAvgContacts(self, arg0: float) -> None:
        """
        Set the average number of contacts a sphere has, before the solver errors out. A huge number can be used to discourage this error type. Defaulted to 100
        """
    def SetErrorOutVelocity(self, arg0: float) -> None:
        """
        Set the velocity which when exceeded, the solver errors out. A huge number can be used to discourage this error type. Defaulted to 5e4.
        """
    def SetExpandFactor(self, beta: float, fix: bool = True) -> None:
        """
        (Explicitly) set the amount by which the radii of the spheres (and the thickness of the boundaries) are expanded for the purpose of contact detection (safe, and creates false positives). If fix is set to true, then this expand factor does not change even if the user uses variable time step size.
        """
    def SetExpandSafetyAdder(self, arg0: float) -> None:
        """
        Set a `base' velocity, which we will always add to our estimated maximum system velocity, when deriving the thinckness of the contact `safety' margin
        """
    def SetExpandSafetyMultiplier(self, arg0: float) -> None:
        """
        Assign a multiplier to our estimated maximum system velocity, when deriving the thinckness of the contact `safety' margin.
        """
    def SetExpandSafetyType(self, arg0: str) -> None:
        """
        A string. If 'auto': the solver automatically derives.
        """
    def SetFamilyClumpMaterial(self, arg0: int, arg1: ...) -> None:
        """
        Set all clumps in this family to have this material.
        """
    def SetFamilyContactWildcardValue(self, arg0: int, arg1: int, arg2: str, arg3: float) -> None:
        """
        Change the value of contact wildcards to val if one of the contact geometry is in family N1, and the other is in N2.
        """
    def SetFamilyContactWildcardValueBoth(self, arg0: int, arg1: str, arg2: float) -> None:
        """
        Change the value of contact wildcards to val if both of the contact geometries are in family N.
        """
    def SetFamilyContactWildcardValueEither(self, arg0: int, arg1: str, arg2: float) -> None:
        """
        Change the value of contact wildcards to val if either of the contact geometries is in family N.
        """
    def SetFamilyExtraMargin(self, arg0: int, arg1: float) -> None:
        """
        Add an extra contact margin to entities in a family so they are registered as potential contact pairs earlier.
        """
    def SetFamilyFixed(self, arg0: int) -> None:
        """
        Mark all entities in this family to be fixed.
        """
    def SetFamilyMeshMaterial(self, arg0: int, arg1: ...) -> None:
        """
        Set all meshes in this family to have this material.
        """
    @typing.overload
    def SetFamilyOwnerWildcardValue(self, arg0: int, arg1: str, arg2: list[float]) -> None:
        """
        Modify the owner wildcard's values of all entities in family N.
        """
    @typing.overload
    def SetFamilyOwnerWildcardValue(self, arg0: int, arg1: str, arg2: float) -> None:
        """
        Modify the owner wildcard's values of all entities in family N.
        """
    @typing.overload
    def SetFamilyPrescribedAngVel(self, ID: int, velX: str, velY: str, velZ: str, dictate: bool = True, pre: str = 'none') -> None:
        """
        Set the prescribed angular velocity to all entities in a family. If dictate is set to true, then this family will not be influenced by the force exerted from other simulation entites (both linear and rotational motions). If false, only specified components (that is, not specified with 'none') will not be influenced by the force exerted from other simulation entites.
        """
    @typing.overload
    def SetFamilyPrescribedAngVel(self, arg0: int) -> None:
        """
        Let the angular velocities of all entites in this family always keep `as is', and not influenced by the force exerted from other simulation entites.
        """
    def SetFamilyPrescribedAngVelX(self, arg0: int) -> None:
        """
        Let the X component of the angular velocities of all entites in this family always keep `as is', and not influenced by the force exerted from other simulation entites.
        """
    def SetFamilyPrescribedAngVelY(self, arg0: int) -> None:
        """
        Let the X component of the angular velocities of all entites in this family always keep `as is', and not influenced by the force exerted from other simulation entites.
        """
    def SetFamilyPrescribedAngVelZ(self, arg0: int) -> None:
        """
        Let the X component of the angular velocities of all entites in this family always keep `as is', and not influenced by the force exerted from other simulation entites.
        """
    @typing.overload
    def SetFamilyPrescribedLinVel(self, ID: int, velX: str, velY: str, velZ: str, dictate: bool = True, pre: str = 'none') -> None:
        """
        Set the prescribed linear velocity to all entities in a family. If dictate is set to true, then this family will not be influenced by the force exerted from other simulation entites (both linear and rotational motions). If false, only specified components (that is, not specified with 'none') will not be influenced by the force exerted from other simulation entites.
        """
    @typing.overload
    def SetFamilyPrescribedLinVel(self, arg0: int) -> None:
        """
        Let the linear velocities of all entites in this family always keep `as is', and not influenced by the force exerted from other simulation entites.
        """
    def SetFamilyPrescribedLinVelX(self, arg0: int) -> None:
        """
        Let the X component of the linear velocities of all entites in this family always keep `as is', and not influenced by the force exerted from other simulation entites.
        """
    def SetFamilyPrescribedLinVelY(self, arg0: int) -> None:
        """
        Let the Y component of the linear velocities of all entites in this family always keep `as is', and not influenced by the force exerted from other simulation entites.
        """
    def SetFamilyPrescribedLinVelZ(self, arg0: int) -> None:
        """
        Let the Z component of the linear velocities of all entites in this family always keep `as is', and not influenced by the force exerted from other simulation entites.
        """
    @typing.overload
    def SetFamilyPrescribedPosition(self, ID: int, X: str, Y: str, Z: str, dictate: bool = True, pre: str = 'none') -> None:
        """
        Keep the positions of all entites in this family to remain exactly the user-specified values.
        """
    @typing.overload
    def SetFamilyPrescribedPosition(self, arg0: int) -> None:
        """
        Keep the positions of all entites in this family to remain as is.
        """
    def SetFamilyPrescribedPositionX(self, arg0: int) -> None:
        """
        Let the X component of the linear positions of all entites in this family always keep `as is'.
        """
    def SetFamilyPrescribedPositionY(self, arg0: int) -> None:
        """
        Let the Y component of the linear positions of all entites in this family always keep `as is'.
        """
    def SetFamilyPrescribedPositionZ(self, arg0: int) -> None:
        """
        Let the Z component of the linear positions of all entites in this family always keep `as is'.
        """
    @typing.overload
    def SetFamilyPrescribedQuaternion(self, ID: int, q_formula: str, dictate: bool = True) -> None:
        """
        Keep the orientation quaternions of all entites in this family to remain exactly the user-specified values.
        """
    @typing.overload
    def SetFamilyPrescribedQuaternion(self, arg0: int) -> None:
        """
        Let the orientation quaternions of all entites in this family always keep `as is'.
        """
    def SetForceCalcThreadsPerBlock(self, arg0: int) -> None:
        """
        Set the number of threads per block in force calculation (default 256).
        """
    def SetGeometryWildcards(self, arg0: set[str]) -> None:
        """
        Set the names for the extra quantities that will be associated with each geometry entity (such as sphere, triangle).
        """
    def SetGravitationalAcceleration(self, arg0: list[float]) -> None:
        """
        Set gravitational pull
        """
    def SetInitBinNumTarget(self, arg0: int) -> None:
        """
        Set the target number of bins (for contact detection) at the start of the simulation upon initialization.
        """
    def SetInitBinSize(self, arg0: float) -> None:
        """
         Explicitly instruct the bin size (for contact detection) that the solver should use.
        """
    def SetInitBinSizeAsMultipleOfSmallestSphere(self, arg0: float) -> None:
        """
        Explicitly instruct the bin size (for contact detection) that the solver should use, as a multiple of the radius of the smallest sphere in simulation.
        """
    def SetInitTimeStep(self, arg0: float) -> None:
        """
        Set the initial time step size. If using constant step size, then this will be used throughout; otherwise, the actual step size depends on the variable step strategy.
        """
    @typing.overload
    def SetIntegrator(self, arg0: str) -> None:
        """
        Set the time integrator for this simulator.
        """
    @typing.overload
    def SetIntegrator(self, arg0: ...) -> None:
        """
        Set the time integrator for this simulator.
        """
    def SetJitifyClumpTemplates(self, use: bool = True) -> None:
        """
        Instruct the solver to rearrange and consolidate clump templates information, then jitify it into GPU kernels (if set to true), rather than using flattened sphere component configuration arrays whose entries are associated with individual spheres.
        """
    def SetJitifyMassProperties(self, use: bool = True) -> None:
        """
        Instruct the solver to rearrange and consolidate mass property information (for all owner types), then jitify it into GPU kernels (if set to true), rather than using flattened mass property arrays whose entries are associated with individual owners.
        """
    def SetJitifyOptions(self, arg0: list[str]) -> None:
        """
        Set the jitification options. It is only needed by advanced users.
        """
    def SetKernelInclude(self, arg0: str) -> None:
        """
        Set the kernels' headers' extra include lines. Useful for customization.
        """
    def SetMaterialPropertyPair(self, arg0: str, arg1: ..., arg2: ..., arg3: float) -> None:
        """
        Set the value for a material property that by nature involves a pair of a materials (e.g. friction coefficient).
        """
    def SetMaxSphereInBin(self, arg0: int) -> None:
        """
        Used to force the solver to error out when there are too many spheres in a bin. A huge number can be used to discourage this error type
        """
    def SetMaxTriangleInBin(self, arg0: int) -> None:
        """
        Used to force the solver to error out when there are too many spheres in a bin. A huge number can be used to discourage this error type
        """
    def SetMaxVelocity(self, arg0: float) -> None:
        """
        Set the maximum expected particle velocity. The solver will not use a velocity larger than this for determining the margin thickness, and velocity larger than this will be considered a system anomaly.
        """
    def SetMeshOutputFormat(self, arg0: str) -> None:
        """
        Specify the output file format of meshes.
        """
    def SetNoForceRecord(self, flag: bool = True) -> None:
        """
        Instruct the solver that there is no need to record the contact force (and contact point location etc.) in an array.
        """
    def SetOutputContent(self, arg0: list[str]) -> None:
        """
        Specify the information that needs to go into the clump or sphere output files.
        """
    def SetOutputFormat(self, arg0: str) -> None:
        """
        Choose sphere and clump output file format.
        """
    def SetOwnerAngVel(self, arg0: int, arg1: list[float3]) -> None:
        """
        Set angular velocity of consecutive owners starting from ownerID, based on input angular velocity vector. N (the size of the input vector) elements will be modified.
        """
    def SetOwnerFamily(self, ownerID: int, fam: int, n: int = 1) -> None:
        """
        Set the family number of consecutive owners.
        """
    def SetOwnerOriQ(self, arg0: int, arg1: list[float4]) -> None:
        """
        Set quaternion of consecutive owners starting from ownerID, based on input quaternion vector. N (the size of the input vector) elements will be modified.
        """
    def SetOwnerPosition(self, arg0: int, arg1: list[float3]) -> None:
        """
        Set position of consecutive owners starting from ownerID, based on input position vector. N (the size of the input vector) elements will be modified.
        """
    def SetOwnerVelocity(self, arg0: int, arg1: list[float3]) -> None:
        """
        Set velocity of consecutive owners starting from ownerID, based on input velocity vector. N (the size of the input vector) elements will be modified.
        """
    @typing.overload
    def SetOwnerWildcardValue(self, ownerIDs: int, name: str, vals: list[float]) -> None:
        """
        Set the wildcard values of some owners using a list.
        """
    @typing.overload
    def SetOwnerWildcardValue(self, ownerIDs: int, name: str, val: float, n: int = 1) -> None:
        """
        Set the wildcard values of some owners using a list.
        """
    def SetOwnerWildcards(self, arg0: set[str]) -> None:
        """
        Set the names for the extra quantities that will be associated with each owner.
        """
    def SetSimTime(self, arg0: float) -> None:
        """
        Get the simulation time passed since the start of simulation.
        """
    def SetSortContactPairs(self, arg0: bool) -> None:
        """
        Instruct the solver if contact pair arrays should be sorted (based on the types of contacts) before usage.
        """
    def SetSphereWildcardValue(self, arg0: int, arg1: str, arg2: list[float]) -> None:
        """
        Set the wildcard values of some spheres.
        """
    def SetTriNodeRelPos(self, arg0: int, arg1: int, arg2: list[float3]) -> None:
        """
        Rewrite the relative positions of the flattened triangle soup.
        """
    def SetTriWildcardValue(self, arg0: int, arg1: str, arg2: list[float]) -> None:
        """
        Set the wildcard values of some triangles.
        """
    def SetVerbosity(self, arg0: str) -> None:
        """
        Set the verbosity level of the solver.
        """
    def ShowAnomalies(self) -> None:
        """
        Show potential anomalies that may have been there in the simulation, then clear the anomaly log.
        """
    def ShowMemStats(self) -> None:
        """
        Print the current memory usage in pretty format.
        """
    def ShowThreadCollaborationStats(self) -> None:
        """
        Show the collaboration stats between dT and kT. This is more useful for tweaking the number of time steps that dT should be allowed to be in advance of kT.
        """
    def ShowTimingStats(self) -> None:
        """
        Show the wall time and percentages of wall time spend on various solver tasks.
        """
    def SyncMemoryTransfer(self) -> None:
        """
        If the user used async-ed version of a tracker's get/set methods (to get a speed boost in many piecemeal accesses of a long array), this method should be called to mark the end of to-host transactions.
        """
    def Track(self, arg0: DEMInitializer) -> Tracker:
        """
        Create a DEMTracker to allow direct control/modification/query to this external object/batch of clumps/triangle mesh object.
        """
    def UpdateClumps(self) -> None:
        """
        Transfer newly loaded clumps to the GPU-side in mid-simulation.
        """
    def UpdateSimParams(self) -> None:
        """
        Transfer the cached sim params to the workers. Used for sim environment modification after system initialization.
        """
    def UpdateStepSize(self, ts: float = -1.0) -> None:
        """
        Update the time step size. Used after system initialization.
        """
    def UpdateTriNodeRelPos(self, arg0: int, arg1: int, arg2: list[float3]) -> None:
        """
        Update the relative positions of the flattened triangle soup.
        """
    def UseAdaptiveBinSize(self, use: bool = True) -> None:
        """
        Enable or disable the use of adaptive bin size (by default it is on)
        """
    def UseAdaptiveUpdateFreq(self, use: bool = True) -> None:
        """
        Enable or disable the use of adaptive max update step count (by default it is on)
        """
    def UseCubForceCollection(self, flag: bool = True) -> None:
        """
        Whether the force collection (acceleration calc and reduction) process should be using CUB. If true, the acceleration array is flattened and reduced using CUB; if false, the acceleration is computed and directly applied to each body through atomic operations.
        """
    def UseFrictionalHertzianModel(self) -> DEMForceModel:
        """
        Instruct the solver to use frictonal (history-based) Hertzian contact force model.
        """
    def UseFrictionlessHertzianModel(self) -> DEMForceModel:
        """
        Instruct the solver to use frictonless Hertzian contact force model
        """
    def WriteClumpFile(self, outfilename: str, accuracy: int = 10) -> None:
        """
        Write the current status of clumps to a file.
        """
    def WriteContactFile(self, outfilename: str, force_thres: float = 1e-30) -> None:
        """
        Write all contact pairs to a file. Forces smaller than threshold will not be outputted.
        """
    def WriteContactFileIncludingPotentialPairs(self, outfilename: str) -> None:
        """
        Write all contact pairs kT-supplied to a file, thus including the potential ones (those are not yet in contact, or recently used to be in contact).
        """
    def WriteMeshFile(self, arg0: str) -> None:
        """
        Write the current status of all meshes to a file.
        """
    def WriteSphereFile(self, arg0: str) -> None:
        """
        Writes the current status of clumps (but decomposed as spheres) file.
        """
    def __init__(self, nGPUs: int = 2) -> None:
        ...
class DEMTrackedObj(DEMInitializer):
    def __init__(self, arg0: DEMTrackedObj) -> None:
        ...
class DataContainer:
    def Get(self, arg0: str) -> list[float]:
        """
        Get a float value from the container by name. Should be used for getting wildcard by name.
        """
    def __init__(self) -> None:
        ...
class FORCE_MODEL:
    """
    Members:
    
      HERTZIAN
    
      HERTZIAN_FRICTIONLES
    
      CUSTOM
    """
    CUSTOM: typing.ClassVar[FORCE_MODEL]  # value = <FORCE_MODEL.CUSTOM: 2>
    HERTZIAN: typing.ClassVar[FORCE_MODEL]  # value = <FORCE_MODEL.HERTZIAN: 0>
    HERTZIAN_FRICTIONLES: typing.ClassVar[FORCE_MODEL]  # value = <FORCE_MODEL.HERTZIAN_FRICTIONLES: 1>
    __members__: typing.ClassVar[dict[str, FORCE_MODEL]]  # value = {'HERTZIAN': <FORCE_MODEL.HERTZIAN: 0>, 'HERTZIAN_FRICTIONLES': <FORCE_MODEL.HERTZIAN_FRICTIONLES: 1>, 'CUSTOM': <FORCE_MODEL.CUSTOM: 2>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class GridSampler:
    def GetSeparation(self) -> float:
        ...
    def SampleBox(self, arg0: list[float], arg1: list[float]) -> list[list[float]]:
        """
        Generates a sample box
        """
    def SampleCylinderX(self, arg0: list[float], arg1: float, arg2: float) -> list[list[float]]:
        ...
    def SampleCylinderY(self, arg0: list[float], arg1: float, arg2: float) -> list[list[float]]:
        ...
    def SampleCylinderZ(self, arg0: list[float], arg1: float, arg2: float) -> list[list[float]]:
        ...
    def SampleSphere(self, arg0: list[float], arg1: float) -> list[list[float]]:
        ...
    def SetSeparation(self, arg0: float) -> None:
        ...
    def __init__(self, arg0: float) -> None:
        ...
class HCPSampler:
    def GetSeparation(self) -> float:
        ...
    def SampleBox(self, arg0: list[float], arg1: list[float]) -> list[list[float]]:
        """
        Generates a sample box
        """
    def SampleCylinderX(self, arg0: list[float], arg1: float, arg2: float) -> list[list[float]]:
        ...
    def SampleCylinderY(self, arg0: list[float], arg1: float, arg2: float) -> list[list[float]]:
        ...
    def SampleCylinderZ(self, arg0: list[float], arg1: float, arg2: float) -> list[list[float]]:
        ...
    def SampleSphere(self, arg0: list[float], arg1: float) -> list[list[float]]:
        ...
    def SetSeparation(self, arg0: float) -> None:
        ...
    def __init__(self, arg0: float) -> None:
        ...
class MESH_FORMAT:
    """
    Members:
    
      VTK
    
      OBJ
    """
    OBJ: typing.ClassVar[MESH_FORMAT]  # value = <MESH_FORMAT.OBJ: 1>
    VTK: typing.ClassVar[MESH_FORMAT]  # value = <MESH_FORMAT.VTK: 0>
    __members__: typing.ClassVar[dict[str, MESH_FORMAT]]  # value = {'VTK': <MESH_FORMAT.VTK: 0>, 'OBJ': <MESH_FORMAT.OBJ: 1>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class OUTPUT_CONTENT:
    """
    Members:
    
      XYZ
    
      QUAT
    
      ABSV
    
      VEL
    
      ANG_VEL
    
      ABS_ACC
    
      ACC
    
      ANG_ACC
    
      FAMILY
    
      MAT
    
      OWNER_WILDCARD
    
      GEO_WILDCARD
    
      EXP_FACTOR
    """
    ABSV: typing.ClassVar[OUTPUT_CONTENT]  # value = <OUTPUT_CONTENT.ABSV: 2>
    ABS_ACC: typing.ClassVar[OUTPUT_CONTENT]  # value = <OUTPUT_CONTENT.ABS_ACC: 16>
    ACC: typing.ClassVar[OUTPUT_CONTENT]  # value = <OUTPUT_CONTENT.ACC: 32>
    ANG_ACC: typing.ClassVar[OUTPUT_CONTENT]  # value = <OUTPUT_CONTENT.ANG_ACC: 64>
    ANG_VEL: typing.ClassVar[OUTPUT_CONTENT]  # value = <OUTPUT_CONTENT.ANG_VEL: 8>
    EXP_FACTOR: typing.ClassVar[OUTPUT_CONTENT]  # value = <OUTPUT_CONTENT.EXP_FACTOR: 2048>
    FAMILY: typing.ClassVar[OUTPUT_CONTENT]  # value = <OUTPUT_CONTENT.FAMILY: 128>
    GEO_WILDCARD: typing.ClassVar[OUTPUT_CONTENT]  # value = <OUTPUT_CONTENT.GEO_WILDCARD: 1024>
    MAT: typing.ClassVar[OUTPUT_CONTENT]  # value = <OUTPUT_CONTENT.MAT: 256>
    OWNER_WILDCARD: typing.ClassVar[OUTPUT_CONTENT]  # value = <OUTPUT_CONTENT.OWNER_WILDCARD: 512>
    QUAT: typing.ClassVar[OUTPUT_CONTENT]  # value = <OUTPUT_CONTENT.QUAT: 1>
    VEL: typing.ClassVar[OUTPUT_CONTENT]  # value = <OUTPUT_CONTENT.VEL: 4>
    XYZ: typing.ClassVar[OUTPUT_CONTENT]  # value = <OUTPUT_CONTENT.XYZ: 0>
    __members__: typing.ClassVar[dict[str, OUTPUT_CONTENT]]  # value = {'XYZ': <OUTPUT_CONTENT.XYZ: 0>, 'QUAT': <OUTPUT_CONTENT.QUAT: 1>, 'ABSV': <OUTPUT_CONTENT.ABSV: 2>, 'VEL': <OUTPUT_CONTENT.VEL: 4>, 'ANG_VEL': <OUTPUT_CONTENT.ANG_VEL: 8>, 'ABS_ACC': <OUTPUT_CONTENT.ABS_ACC: 16>, 'ACC': <OUTPUT_CONTENT.ACC: 32>, 'ANG_ACC': <OUTPUT_CONTENT.ANG_ACC: 64>, 'FAMILY': <OUTPUT_CONTENT.FAMILY: 128>, 'MAT': <OUTPUT_CONTENT.MAT: 256>, 'OWNER_WILDCARD': <OUTPUT_CONTENT.OWNER_WILDCARD: 512>, 'GEO_WILDCARD': <OUTPUT_CONTENT.GEO_WILDCARD: 1024>, 'EXP_FACTOR': <OUTPUT_CONTENT.EXP_FACTOR: 2048>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class OUTPUT_FORMAT:
    """
    Members:
    
      CSV
    
      BINARY
    
      CHPF
    """
    BINARY: typing.ClassVar[OUTPUT_FORMAT]  # value = <OUTPUT_FORMAT.BINARY: 1>
    CHPF: typing.ClassVar[OUTPUT_FORMAT]  # value = <OUTPUT_FORMAT.CHPF: 2>
    CSV: typing.ClassVar[OUTPUT_FORMAT]  # value = <OUTPUT_FORMAT.CSV: 0>
    __members__: typing.ClassVar[dict[str, OUTPUT_FORMAT]]  # value = {'CSV': <OUTPUT_FORMAT.CSV: 0>, 'BINARY': <OUTPUT_FORMAT.BINARY: 1>, 'CHPF': <OUTPUT_FORMAT.CHPF: 2>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class OWNER_TYPE:
    """
    Members:
    
      CLUMP
    
      ANALYTICAL
    
      MESH
    """
    ANALYTICAL: typing.ClassVar[OWNER_TYPE]  # value = <OWNER_TYPE.ANALYTICAL: 1>
    CLUMP: typing.ClassVar[OWNER_TYPE]  # value = <OWNER_TYPE.CLUMP: 0>
    MESH: typing.ClassVar[OWNER_TYPE]  # value = <OWNER_TYPE.MESH: 2>
    __members__: typing.ClassVar[dict[str, OWNER_TYPE]]  # value = {'CLUMP': <OWNER_TYPE.CLUMP: 0>, 'ANALYTICAL': <OWNER_TYPE.ANALYTICAL: 1>, 'MESH': <OWNER_TYPE.MESH: 2>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class PDSampler:
    def GetSeparation(self) -> float:
        ...
    def SampleBox(self, arg0: list[float], arg1: list[float]) -> list[list[float]]:
        """
        Generates a sample box
        """
    def SampleCylinderX(self, arg0: list[float], arg1: float, arg2: float) -> list[list[float]]:
        ...
    def SampleCylinderY(self, arg0: list[float], arg1: float, arg2: float) -> list[list[float]]:
        ...
    def SampleCylinderZ(self, arg0: list[float], arg1: float, arg2: float) -> list[list[float]]:
        ...
    def SampleSphere(self, arg0: list[float], arg1: float) -> list[list[float]]:
        ...
    def SetRandomEngineSeed(self, arg0: int) -> None:
        ...
    def SetSeparation(self, arg0: float) -> None:
        ...
    def __init__(self, arg0: float) -> None:
        ...
class RuntimeDataHelper:
    @staticmethod
    def SetCUDAToolkitHeaders(arg0: str) -> None:
        ...
    @staticmethod
    def SetCUDAToolkitTargetHeaders(arg0: str) -> None:
        ...
    @staticmethod
    def SetPathPrefix(arg0: str) -> None:
        ...
    def __init__(self) -> None:
        ...
class SPATIAL_DIR:
    """
    Members:
    
      X
    
      Y
    
      Z
    
      NONE
    """
    NONE: typing.ClassVar[SPATIAL_DIR]  # value = <SPATIAL_DIR.NONE: 3>
    X: typing.ClassVar[SPATIAL_DIR]  # value = <SPATIAL_DIR.X: 0>
    Y: typing.ClassVar[SPATIAL_DIR]  # value = <SPATIAL_DIR.Y: 1>
    Z: typing.ClassVar[SPATIAL_DIR]  # value = <SPATIAL_DIR.Z: 2>
    __members__: typing.ClassVar[dict[str, SPATIAL_DIR]]  # value = {'X': <SPATIAL_DIR.X: 0>, 'Y': <SPATIAL_DIR.Y: 1>, 'Z': <SPATIAL_DIR.Z: 2>, 'NONE': <SPATIAL_DIR.NONE: 3>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class TIME_INTEGRATOR:
    """
    Members:
    
      FORWARD_EULER
    
      CENTERED_DIFFERENCE
    
      EXTENDED_TAYLOR
    
      CHUNG
    """
    CENTERED_DIFFERENCE: typing.ClassVar[TIME_INTEGRATOR]  # value = <TIME_INTEGRATOR.CENTERED_DIFFERENCE: 1>
    CHUNG: typing.ClassVar[TIME_INTEGRATOR]  # value = <TIME_INTEGRATOR.CHUNG: 3>
    EXTENDED_TAYLOR: typing.ClassVar[TIME_INTEGRATOR]  # value = <TIME_INTEGRATOR.EXTENDED_TAYLOR: 2>
    FORWARD_EULER: typing.ClassVar[TIME_INTEGRATOR]  # value = <TIME_INTEGRATOR.FORWARD_EULER: 0>
    __members__: typing.ClassVar[dict[str, TIME_INTEGRATOR]]  # value = {'FORWARD_EULER': <TIME_INTEGRATOR.FORWARD_EULER: 0>, 'CENTERED_DIFFERENCE': <TIME_INTEGRATOR.CENTERED_DIFFERENCE: 1>, 'EXTENDED_TAYLOR': <TIME_INTEGRATOR.EXTENDED_TAYLOR: 2>, 'CHUNG': <TIME_INTEGRATOR.CHUNG: 3>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class Tracker:
    @typing.overload
    def AddAcc(self, acc: float3, offset: int = 0) -> None:
        """
        Add an extra acc to the tracked body, for the next time step. Note if the user intends to add a persistent external force, then using family prescription is the better method.
        """
    @typing.overload
    def AddAcc(self, acc: list[float3]) -> None:
        """
        Add an extra acc to consecutive tracked objects, (only) for the next time step. Note if the user intends to add a persistent external force, then using family prescription is the better method.
        """
    @typing.overload
    def AddAngAcc(self, angAcc: float3, offset: int = 0) -> None:
        """
        Add an extra angular acceleration to the tracked body, for the next time step. Note if the user intends to add a persistent external torque, then using family prescription is the better method.
        """
    @typing.overload
    def AddAngAcc(self, angAcc: list[float3]) -> None:
        """
        Add an extra angular acceleration to consecutive tracked objects, (only) for the next time step. Note if the user intends to add a persistent external torque, then using family prescription is the better method.
        """
    def AngVelGlobal(self, offset: int = 0) -> list[float]:
        """
        Get the angular velocity of this tracked object in global coordinate system.
        """
    def AngVelLocal(self, offset: int = 0) -> list[float]:
        """
        Get the angular velocity of this tracked object in its own local coordinate system. Applying OriQ to it would give you the ang vel in global frame.
        """
    def AngularVelocitiesGlobal(self) -> list[list[float]]:
        """
        Get the angular velocity of all objects tracked by this tracker, in global coordinate system.
        """
    def AngularVelocitiesLocal(self) -> list[list[float]]:
        """
        Get the angular velocity of all tracked objects in their own local coordinate system. Applying OriQ to it would give you the ang vel in global frame.
        """
    def ContactAcc(self, offset: int = 0) -> list[float]:
        """
        Get the a portion of the acceleration of this tracked object, that is the result of its contact with other simulation entities. In most cases, this means excluding the gravitational acceleration. The acceleration is in global frame.
        """
    def ContactAccelerations(self) -> list[list[float]]:
        """
        Get the acceleration experienced by all objects tracked by this tracker, that is the result of their contact with other simulation entities. The acceleration is in global frame. In most cases, this means excluding the gravitational acceleration. The acceleration is in global frame.
        """
    def ContactAngAccGlobal(self, offset: int = 0) -> list[float]:
        """
        Get the a portion of the angular acceleration of this tracked object, that is the result of its contact with other simulation entities. The acceleration is in this object's global frame.
        """
    def ContactAngAccLocal(self, offset: int = 0) -> list[float]:
        """
        Get the a portion of the angular acceleration of this tracked object, that is the result of its contact with other simulation entities. The acceleration is in this object's local frame.
        """
    def ContactAngularAccelerationsGlobal(self) -> list[list[float]]:
        """
        Get the angular acceleration experienced by all objects tracked by this tracker, that is the result of their contact with other simulation entities. The acceleration is in this object's global frame.
        """
    def ContactAngularAccelerationsLocal(self) -> list[list[float]]:
        """
        Get the angular acceleration experienced by all objects tracked by this tracker, that is the result of their contact with other simulation entities. The acceleration is in this object's local frame.
        """
    def GetContactClumps(self, offset: int = 0) -> list[int]:
        """
        Get the clumps that are in contact with this tracked owner as a vector.
        """
    def GetContactForces(self, offset: int = 0) -> list[list[list[float]]]:
        """
        Get all contact forces that concern this tracked object, as a list.
        """
    def GetContactForcesAndGlobalTorque(self, offset: int = 0) -> list[list[list[float]]]:
        """
        Get all contact forces and global torques that concern this tracked object, as a list.
        """
    def GetContactForcesAndGlobalTorqueForAll(self) -> list[list[list[float]]]:
        """
        Get all contact forces and global torques that concern all objects tracked by this tracker, as a list.
        """
    def GetContactForcesAndLocalTorque(self, offset: int = 0) -> list[list[list[float]]]:
        """
        Get all contact forces and local torques that concern this tracked object, as a list.
        """
    def GetContactForcesAndLocalTorqueForAll(self) -> list[list[list[float]]]:
        """
        Get all contact forces and local torques that concern all objects tracked by this tracker, as a list.
        """
    def GetContactForcesForAll(self) -> list[list[list[float]]]:
        """
        Get all contact forces that concern all objects tracked by this tracker, as a list.
        """
    def GetFamilies(self) -> list[int]:
        """
        Get the family numbers of all the tracked object.
        """
    def GetFamily(self, offset: int = 0) -> int:
        """
        Get the family number of the tracked object.
        """
    def GetGeometryWildcardValue(self, name: str, offset: int) -> float:
        """
        Get the geometry wildcard values for a geometry entity tracked by this tracker.
        """
    def GetGeometryWildcardValues(self, name: str) -> list[float]:
        """
        Get the geometry wildcard values for all the geometry entities tracked by this tracker.
        """
    def GetMesh(self) -> ...:
        """
        Get a handle for the mesh this tracker is tracking.
        """
    def GetMeshNodesGlobal(self) -> list[float3]:
        """
        Get the current locations of all the nodes in the mesh being tracked.
        """
    def GetOwnerID(self, offset: int = 0) -> int:
        """
        Get the owner ID of the tracked obj.
        """
    def GetOwnerIDs(self) -> list[int]:
        """
        Get the owner IDs of all the tracked objects.
        """
    def GetOwnerWildcardValue(self, name: str, offset: int = 0) -> float:
        """
        Get the owner's wildcard value.
        """
    def GetOwnerWildcardValues(self, name: str) -> list[float]:
        """
        Get the owner wildcard values for all the owners entities tracked by this tracker.
        """
    def MOI(self, offset: int = 0) -> list[float]:
        """
        Get the moment of inertia (in principal axis frame) of the tracked object.
        """
    def MOIs(self) -> list[list[float]]:
        """
        Get the moment of inertia (in principal axis frame) of all the tracked objects.
        """
    def Mass(self, offset: int = 0) -> float:
        """
        Get the mass of the tracked object.
        """
    def Masses(self) -> list[float]:
        """
        Get the masses of all the tracked objects.
        """
    def OriQ(self, offset: int = 0) -> list[float]:
        """
        Get the quaternion that represents the orientation of this tracked object's own coordinate system. Returns a vector of 4 floats. The order is (x, y, z, w). If compared against Chrono naming convention, then it is saying our ordering here is (e1, e2, e3, e0).
        """
    def OrientationQuaternions(self) -> list[list[float]]:
        """
        Get all quaternions that represent the orientation of all the tracked objects' own coordinate systems. Returns a vector of 4-float vectors. The order is (x, y, z, w). If compared against Chrono naming convention, then it is saying our ordering here is (e1, e2, e3, e0).
        """
    def Pos(self, offset: int = 0) -> list[float]:
        """
        Get the position of this tracked object.
        """
    def Positions(self) -> list[list[float]]:
        """
        Get the positions of all tracked objects.
        """
    @typing.overload
    def SetAngVel(self, angVel: float3, offset: int = 0) -> None:
        """
        Set the angular velocity of this tracked object in its own local coordinate system.
        """
    @typing.overload
    def SetAngVel(self, angVel: list[float3]) -> None:
        """
        Set the angular velocity of consecutive tracked objects in their own local coordinate systems.
        """
    @typing.overload
    def SetFamily(self, fam_num: int) -> None:
        """
        Change the family numbers of all the entities tracked by this tracker.
        """
    @typing.overload
    def SetFamily(self, fam_num: int, offset: int) -> None:
        """
        Change the family number of one entities tracked by this tracker.
        """
    def SetGeometryWildcardValue(self, name: str, wc: float, offset: int = 0) -> None:
        """
        Set a wildcard value of the geometry entity this tracker is tracking.
        """
    def SetGeometryWildcardValues(self, name: str, wc: list[float]) -> None:
        """
        Set a wildcard value of the geometry entities this tracker is tracking.
        """
    @typing.overload
    def SetOriQ(self, oriQ: float4, offset: int = 0) -> None:
        """
        Set the quaternion which represents the orientation of this tracked object's coordinate system.
        """
    @typing.overload
    def SetOriQ(self, oriQ: list[float4]) -> None:
        """
        Set the quaternion which represents the orientation of consecutive tracked objects' coordinate systems.
        """
    def SetOwnerWildcardValue(self, name: str, wc: float, offset: int = 0) -> None:
        """
        Set a wildcard value of the owner this tracker is tracking.
        """
    def SetOwnerWildcardValues(self, name: str, wc: list[float]) -> None:
        """
        Set a wildcard value of the owner this tracker is tracking.
        """
    @typing.overload
    def SetPos(self, pos: float3, offset: int = 0) -> None:
        """
        Set the position of this tracked object.
        """
    @typing.overload
    def SetPos(self, pos: list[float3]) -> None:
        """
        Set the positions of consecutive tracked objects.
        """
    @typing.overload
    def SetVel(self, vel: float3, offset: int = 0) -> None:
        """
        Set the velocity of this tracked object in global frame.
        """
    @typing.overload
    def SetVel(self, vel: list[float3]) -> None:
        """
        Set the velocity of consecutive tracked objects in global frame.
        """
    def UpdateMesh(self, new_nodes: list[float3]) -> None:
        """
        Apply the new mesh node positions such that the tracked mesh is replaced by the new_nodes.
        """
    def UpdateMeshByIncrement(self, deformation: list[float3]) -> None:
        """
        Change the coordinates of each mesh node by the given amount.
        """
    def Vel(self, offset: int = 0) -> list[float]:
        """
        Get the velocity of this tracked object in global frame.
        """
    def Velocities(self) -> list[list[float]]:
        """
        Get the velocities of all objects tracked by this tracker, in global frame.
        """
    def __init__(self, arg0: ...) -> None:
        ...
class VERBOSITY:
    """
    Members:
    
      QUIET
    
      ERROR
    
      WARNING
    
      INFO
    
      STEP_ANOMALY
    
      STEP_METRIC
    
      DEBUG
    
      STEP_DEBUG
    """
    DEBUG: typing.ClassVar[VERBOSITY]  # value = <VERBOSITY.DEBUG: 40>
    ERROR: typing.ClassVar[VERBOSITY]  # value = <VERBOSITY.ERROR: 10>
    INFO: typing.ClassVar[VERBOSITY]  # value = <VERBOSITY.INFO: 30>
    QUIET: typing.ClassVar[VERBOSITY]  # value = <VERBOSITY.QUIET: 0>
    STEP_ANOMALY: typing.ClassVar[VERBOSITY]  # value = <VERBOSITY.STEP_ANOMALY: 32>
    STEP_DEBUG: typing.ClassVar[VERBOSITY]  # value = <VERBOSITY.STEP_DEBUG: 50>
    STEP_METRIC: typing.ClassVar[VERBOSITY]  # value = <VERBOSITY.STEP_METRIC: 35>
    WARNING: typing.ClassVar[VERBOSITY]  # value = <VERBOSITY.WARNING: 20>
    __members__: typing.ClassVar[dict[str, VERBOSITY]]  # value = {'QUIET': <VERBOSITY.QUIET: 0>, 'ERROR': <VERBOSITY.ERROR: 10>, 'WARNING': <VERBOSITY.WARNING: 20>, 'INFO': <VERBOSITY.INFO: 30>, 'STEP_ANOMALY': <VERBOSITY.STEP_ANOMALY: 32>, 'STEP_METRIC': <VERBOSITY.STEP_METRIC: 35>, 'DEBUG': <VERBOSITY.DEBUG: 40>, 'STEP_DEBUG': <VERBOSITY.STEP_DEBUG: 50>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
def DEMBoxGridSampler(BoxCenter: list[float], HalfDims: list[float], GridSizeX: float, GridSizeY: float = -1.0, GridSizeZ: float = -1.0) -> list[list[float]]:
    ...
def DEMBoxHCPSampler(arg0: list[float], arg1: list[float], arg2: float) -> list[list[float]]:
    ...
def DEMCylSurfSampler(CylCenter: list[float], CylAxis: list[float], CylRad: float, CylHeight: float, ParticleRad: float, spacing: float = 1.2) -> list[list[float]]:
    ...
def FrameTransformGlobalToLocal(arg0: list[float], arg1: list[float], arg2: list[float]) -> list[float]:
    """
    Translating the inverse of the provided vec then applying a local inverse rotation of the provided rot_Q, then return the result.
    """
def FrameTransformLocalToGlobal(arg0: list[float], arg1: list[float], arg2: list[float]) -> list[float]:
    """
    Apply a local rotation then a translation, then return the result.
    """
def GetDEMEDataFile(arg0: str) -> str:
    ...
ABSV: OUTPUT_CONTENT  # value = <OUTPUT_CONTENT.ABSV: 2>
ABS_ACC: OUTPUT_CONTENT  # value = <OUTPUT_CONTENT.ABS_ACC: 16>
ACC: OUTPUT_CONTENT  # value = <OUTPUT_CONTENT.ACC: 32>
ANALYTICAL: OWNER_TYPE  # value = <OWNER_TYPE.ANALYTICAL: 1>
ANG_ACC: OUTPUT_CONTENT  # value = <OUTPUT_CONTENT.ANG_ACC: 64>
ANG_VEL: OUTPUT_CONTENT  # value = <OUTPUT_CONTENT.ANG_VEL: 8>
BINARY: OUTPUT_FORMAT  # value = <OUTPUT_FORMAT.BINARY: 1>
CENTERED_DIFFERENCE: TIME_INTEGRATOR  # value = <TIME_INTEGRATOR.CENTERED_DIFFERENCE: 1>
CHPF: OUTPUT_FORMAT  # value = <OUTPUT_FORMAT.CHPF: 2>
CHUNG: TIME_INTEGRATOR  # value = <TIME_INTEGRATOR.CHUNG: 3>
CLUMP: OWNER_TYPE  # value = <OWNER_TYPE.CLUMP: 0>
CNT_TYPE: CNT_OUTPUT_CONTENT  # value = <CNT_OUTPUT_CONTENT.CNT_TYPE: 0>
CNT_WILDCARD: CNT_OUTPUT_CONTENT  # value = <CNT_OUTPUT_CONTENT.CNT_WILDCARD: 32>
COMPONENT: CNT_OUTPUT_CONTENT  # value = <CNT_OUTPUT_CONTENT.COMPONENT: 4>
CSV: OUTPUT_FORMAT  # value = <OUTPUT_FORMAT.CSV: 0>
CUSTOM: FORCE_MODEL  # value = <FORCE_MODEL.CUSTOM: 2>
DEBUG: VERBOSITY  # value = <VERBOSITY.DEBUG: 40>
ERROR: VERBOSITY  # value = <VERBOSITY.ERROR: 10>
EXP_FACTOR: OUTPUT_CONTENT  # value = <OUTPUT_CONTENT.EXP_FACTOR: 2048>
EXTENDED_TAYLOR: TIME_INTEGRATOR  # value = <TIME_INTEGRATOR.EXTENDED_TAYLOR: 2>
FAMILY: OUTPUT_CONTENT  # value = <OUTPUT_CONTENT.FAMILY: 128>
FORCE: CNT_OUTPUT_CONTENT  # value = <CNT_OUTPUT_CONTENT.FORCE: 1>
FORWARD_EULER: TIME_INTEGRATOR  # value = <TIME_INTEGRATOR.FORWARD_EULER: 0>
GEO_ID: CNT_OUTPUT_CONTENT  # value = <CNT_OUTPUT_CONTENT.GEO_ID: 128>
GEO_WILDCARD: OUTPUT_CONTENT  # value = <OUTPUT_CONTENT.GEO_WILDCARD: 1024>
HERTZIAN: FORCE_MODEL  # value = <FORCE_MODEL.HERTZIAN: 0>
HERTZIAN_FRICTIONLES: FORCE_MODEL  # value = <FORCE_MODEL.HERTZIAN_FRICTIONLES: 1>
INFO: VERBOSITY  # value = <VERBOSITY.INFO: 30>
MAT: OUTPUT_CONTENT  # value = <OUTPUT_CONTENT.MAT: 256>
MESH: OWNER_TYPE  # value = <OWNER_TYPE.MESH: 2>
NICKNAME: CNT_OUTPUT_CONTENT  # value = <CNT_OUTPUT_CONTENT.NICKNAME: 256>
NONE: SPATIAL_DIR  # value = <SPATIAL_DIR.NONE: 3>
NORMAL: CNT_OUTPUT_CONTENT  # value = <CNT_OUTPUT_CONTENT.NORMAL: 8>
OBJ: MESH_FORMAT  # value = <MESH_FORMAT.OBJ: 1>
OWNER: CNT_OUTPUT_CONTENT  # value = <CNT_OUTPUT_CONTENT.OWNER: 64>
OWNER_WILDCARD: OUTPUT_CONTENT  # value = <OUTPUT_CONTENT.OWNER_WILDCARD: 512>
PI: float = 3.141592653589793
POINT: CNT_OUTPUT_CONTENT  # value = <CNT_OUTPUT_CONTENT.POINT: 2>
QUAT: OUTPUT_CONTENT  # value = <OUTPUT_CONTENT.QUAT: 1>
QUIET: VERBOSITY  # value = <VERBOSITY.QUIET: 0>
STEP_ANOMALY: VERBOSITY  # value = <VERBOSITY.STEP_ANOMALY: 32>
STEP_DEBUG: VERBOSITY  # value = <VERBOSITY.STEP_DEBUG: 50>
STEP_METRIC: VERBOSITY  # value = <VERBOSITY.STEP_METRIC: 35>
TORQUE: CNT_OUTPUT_CONTENT  # value = <CNT_OUTPUT_CONTENT.TORQUE: 16>
VEL: OUTPUT_CONTENT  # value = <OUTPUT_CONTENT.VEL: 4>
VTK: MESH_FORMAT  # value = <MESH_FORMAT.VTK: 0>
WARNING: VERBOSITY  # value = <VERBOSITY.WARNING: 20>
X: SPATIAL_DIR  # value = <SPATIAL_DIR.X: 0>
XYZ: OUTPUT_CONTENT  # value = <OUTPUT_CONTENT.XYZ: 0>
Y: SPATIAL_DIR  # value = <SPATIAL_DIR.Y: 1>
Z: SPATIAL_DIR  # value = <SPATIAL_DIR.Z: 2>
