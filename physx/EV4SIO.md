# PhysX pour Ev4sio.Native

## Base et organisation

Base NVIDIA : **PhysX 5.13.0**, commit
`7d6f6ac8b83e1610bdf715f1f2f1d6b718aee586` (`ovphysx-0.8.0`).
Le numéro du paquet ovphysx diffère du numéro du SDK PhysX.

La branche `main` intègre NVIDIA et les outils de génération déjà présents dans
le fork. La branche `ev4sio` ajoute les adaptations selon les trois changelists
du document « Adaptation des sources PhysX pour Ev4sio.Native.pdf » :

1. Isolation des namespaces et des symboles globaux.
2. Adaptations de l'API, Blast et véhicules.
3. Génération et compilation Windows, puis validation d'une DLL statique CPU/GPU.

Les mentions de copyright et les licences NVIDIA restent dans les sources.

## Isolation des symboles

Les namespaces `physx`, `Sc`, `Gu`, `Cm`, `Dy`, `Bp`, `Sq`, `IG` et `Ext`
portent le préfixe `ev4sio_`. Les points d'entrée C et les fonctions globales
désignées dans le PDF portent aussi ce préfixe ; la macro de version devient
`ev4sio_PX_PHYSICS_VERSION`.
Cela couvre notamment les nouvelles fonctions immediate mode, cooking,
déformables et GPU. Les chaînes de chargement des fonctions GPU sont adaptées.

`tools/ev4sio_symbols.json` est le manifeste des noms. Les fonctions internes
de GuCooking, GuFactory et ExtTetSplitting utilisent une liaison C++ : leur
namespace renommé isole leurs symboles sans modifier les méthodes publiques
homonymes, telles que `PxPhysics::createTriangleMesh()`.

Depuis la racine du dépôt Git :

```powershell
python physx/tools/ev4sio_prefix.py --check
# Après une nouvelle intégration amont, examiner les changements produits :
python physx/tools/ev4sio_prefix.py
git diff
```

Le script traite seulement les fichiers suivis par Git et refuse un nouveau
point d'entrée C interne non recensé. Ajouter les nouvelles sources à Git avant
le contrôle. Les dossiers de compilation ne sont pas des sources à préfixer.

## API et véhicules

`PxGpuDynamicsMemoryConfig::isValid()` est inline. Le destructeur de
`ev4sio_physx::ev4sio_Ext::JointData` est public. Les entrées GPU requises
restent disponibles ; leur suppression n'est pas un moyen de résoudre les
conflits d'édition des liens.

Blast expose `convexHullApproximation` (false), `planeDownsampling` (4),
`convexHullDownsampling` (4) et `principalComponentAnalysis` (false), transmis
à VHACD. L'accélération OpenCL reste désactivée. Les bibliothèques SDK Blast
sont statiques ; ses tests Premake sont activables par `--build-tests`.

L'API véhicules 4.1 n'existe plus en amont. Les callbacks du fork historique
`Ev4sio/PhysX-4.1.2` (commits `76db61bb` et `2244410e`) sont portés sur l'API
actuelle, dans `vehicle/PxVehicleEv4sioCallbacks.h` :

- `PxVehicleTireForceParams::tireForceShader` et `tireForceShaderData` définissent
  le modèle de pneu par roue. Les composants standards transmettent les vitesses
  longitudinale/latérale, la vitesse de rotation et la gravité. Sans callback,
  le calcul NVIDIA reste utilisé. Pour un appel direct à
  `PxVehicleTireForcesUpdate`, fournir les trois nouveaux arguments finaux.
- `ev4sio_PxVehicleComputeTireForceDefault` accepte les paramètres actuels
  `PxVehicleTireForceParams`, avec leurs unités et convention de carrossage.
  Il ne reçoit pas l'ancien `PxVehicleTireData`. Un pointeur de données nul
  transmet automatiquement les paramètres de pneu au callback.
- `PxVehiclePhysXRoadGeometryQueryParams::suspensionSweepHitShader` peut modifier
  le contact avant le calcul du plan de route, y compris après récupération d'un
  chevauchement initial. `suspensionSweepHitShaderData` est partagé ; le tableau
  optionnel `suspensionSweepHitShaderDataEntries`, indexé par ID de roue, le
  remplace. La largeur reçue est la largeur complète. Les raycasts n'appellent
  pas ce callback. Les données doivent rester valides pendant la simulation.

Les pointeurs de callback sont des données d'exécution et ne sont pas sérialisés.
Le code consommateur doit être recompilé. Il n'y a pas de compatibilité binaire
avec 4.1 ou 5.6. En 5.13, utiliser les en-têtes `vehicle/...`, le namespace
`ev4sio_physx` et `PhysXVehicle_static_64.lib` : les anciens chemins/noms
`vehicle2` ne sont plus présents.

## Construction Windows

Prérequis : Visual Studio 2026 (outils C++), CMake 4.2 minimum, CUDA 13.3 pour le
preset GPU. La validation de cette mise à jour utilise CMake 4.4.0, MSVC 19.51
et CUDA 13.3.33. Packman récupère les dépendances des presets vc18.

```powershell
cd physx
.\generate_projects.bat vc18win64
cmake --build compiler/vc18win64 --config checked --target PhysX PhysXGpu PhysXExtensions PhysXCharacterKinematic PhysXVehicle --parallel 2

# DLL et tests d'intégration facultatifs
cmake -S compiler/public -B compiler/vc18win64 -DPX_BUILD_EV4SIO_TESTS=ON
cmake --build compiler/vc18win64 --config checked --target Ev4sioSmoke --parallel 2
ctest --test-dir compiler/vc18win64 -C checked --output-on-failure
```

Le preset par défaut est `vc18win64`. Le preset `vc18win64-cpu-only` permet de
construire sans CUDA. Les bibliothèques sont statiques avec runtime MSVC `/MD`
(`/MDd` en debug). Aligner le runtime et la configuration du plugin consommateur.
Les chemins CUDA contenant des espaces sont acceptés ; un `PM_CUDA_PATH`
inexistant est écarté au profit d'un `CUDA_PATH` valide. La génération conserve
les compilations incrémentales et propage les erreurs au processus appelant.

`PX_PHYSX_GPU_STATIC` est défini dès la configuration des cibles CPU, avant les
cibles GPU. Cela exclut les wrappers de chargement dynamique CPU lorsque les
fonctions correspondantes sont liées depuis PhysXGpu statique. Aucune option
`/FORCE:MULTIPLE` n'est utilisée. CUDA 13 exclut l'architecture Volta (compute 70),
emploie la nouvelle signature de `cuCtxCreate` et conserve les cibles GPU récentes.

Pour une DLL CMake qui consomme PhysXGpu statique, définir
`set_property(TARGET MonPlugin PROPERTY CUDA_RUNTIME_LIBRARY None)`.
PhysX fournit déjà les points d'entrée de lancement/enregistrement CUDA dans
`CudaKernelWrangler` ; le `cudart_static` ajouté implicitement par CMake produirait
des symboles en double. La DLL de test applique ce réglage. Ce choix concerne
le runtime CUDA, indépendamment du runtime C++ MSVC `/MD`.

Les nouveaux symboles internes `__cudaGetKernel` et `__cudaLaunchKernel` émis
par CUDA 13.3 sont présents pour la liaison des stubs générés ; ils refusent
les lancements runtime. PhysX charge les modules et exécute les noyaux avec son
API driver existante. La fonction plane/convexe de `GuConvexSupport.cpp` est
déplacée inline dans le header commun CPU/CUDA, ce qui évite une seconde
définition hôte dans la même DLL. Son calcul est conservé.

Sous Windows, le `cuda.lib` de CUDA 13.3 demande `LIBCMT`. Pour les builds `/MD`,
la cible PhysXGpu transmet `/NODEFAULTLIB:LIBCMT` afin que ce chargeur utilise
le même CRT dynamique que le plugin. Un projet Visual Studio consommant les
archives directement doit reprendre ce réglage, ainsi que les dépendances
de la cible, sans ajouter `cudart_static.lib`.

## Validation et limites

Validation du 23 novembre 2026 : presets `vc18win64-cpu-only` et `vc18win64`
en configuration `checked`, tests réussis. La scène GPU a été exécutée sur
une GeForce RTX 3060 (pilote 610.88), avec zéro erreur PhysX. L'audit des huit
archives statiques ne retrouve aucun ancien point d'entrée du manifeste ni
namespace PhysX non renommé ; les trois wrappers CUDA examinés sont définis
uniquement dans PhysXGpu. La DLL CPU/GPU exporte uniquement le test.

Le test `Ev4sio.SdkIntegration` lie une DLL avec les bibliothèques statiques
CPU/GPU, puis vérifie une chute sur un plan, le cooking convexe, immediate mode,
les callbacks de pneus et les sweeps de suspension, dont un chevauchement initial.
Il compile aussi le fichier Blast qui transmet les paramètres à VHACD.
Si un périphérique CUDA est disponible, il crée un contexte et exécute une scène
GPU. Sinon, il affiche explicitement que la partie GPU n'a pas été exécutée.

Contrôler les exports de `Ev4sioSmokeDll.dll` avec `dumpbin /exports` : seul le
point d'entrée de test doit être exporté. Le test ne remplace pas la compilation
du vrai projet Ev4sio.Native, ni son chargement aux côtés du PhysX de Unity.
Les projets complets Blast, Flow et ovphysx ne font pas partie de ce test SDK.

Pour la prochaine mise à jour, intégrer d'abord NVIDIA sur `main`, reprendre
les adaptations sur `ev4sio`, contrôler le manifeste, puis refaire une génération
propre, la compilation et les tests avant publication.
