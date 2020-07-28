#include "pclviewerxmodel.h"

/*!
 * \brief PCLViewerXModel::PCLViewerXModel
 * A PCLViewerX-hez tartozó model konstruktora. Inicializálja a attribútumokat
 */
PCLViewerXModel::PCLViewerXModel()
{
    _src.reset (new PointCloudT);
    _tgt.reset (new PointCloudT);
    _preprocessed_src.reset(new PointCloudT);
    _preprocessed_tgt.reset(new PointCloudT);
    _keypoints_src.reset (new PointCloudT);
    _keypoints_tgt.reset (new PointCloudT);
    _output.reset(new PointCloudT);
    _transform.setIdentity();
    _correspondences.reset(new pcl::Correspondences);
}

/*!
 * \brief PCLViewerXModel::~PCLViewerXModel
 * A PCLViewerXModel-hez tartozó konstruktor
 */
PCLViewerXModel::~PCLViewerXModel()
{

}


/*!
 * \brief PCLViewerXModel::DeletePreprocessedSource
 * Az előfeldolgozott forrás felhő törléséért felelős függvény. Miután az ezeket befolyásoló értékeket is resetelte az eredeti forrás felhő tartalmát
 * belemásolja az "Előfeldolgozott" felhőbe
 */
void PCLViewerXModel::DeletePreprocessedSource()
{
    _preprocessed_src.reset(new PointCloudT);
    _output.reset(new PointCloudT);
    _keypoints_src.reset(new PointCloudT);
    _keypoints_tgt.reset(new PointCloudT);
    _correspondences.reset(new pcl::Correspondences);

    pcl::copyPointCloud(*_src,*_preprocessed_src);
}

/*!
 * \brief PCLViewerXModel::DeletePreprocessedTarget
 * Az előfeldolgozott cél felhő törléséért felelős függvény. Miután az ezeket befolyásoló értékeket is resetelte az eredeti cél felhő tartalmát
 * belemásolja az "Előfeldolgozott" felhőbe
 */
void PCLViewerXModel::DeletePreprocessedTarget()
{
    _preprocessed_tgt.reset(new PointCloudT);
    _keypoints_tgt.reset(new PointCloudT);
    _keypoints_src.reset(new PointCloudT);
    pcl::copyPointCloud(*_tgt,*_preprocessed_tgt);
    _output.reset(new PointCloudT);
    _correspondences.reset(new pcl::Correspondences);
}

/*!
 * \brief PCLViewerXModel::LoadTgtPointCloud
 * \param path - a megnyitni kívánt fájl útvonala
 * - a fájlmegnyitásnak az állapota. 0 ,ha helyes volt a betöltés, negatív ha hibás.
 * A cél felhő betöltéséért felelős metódus. Megkapja a megnyitni kívánt fájl útvonalát, majd a célhez tartozó felhőbe másolja a tartalmát a megfelelő
 * felhő betöltő metódussal. Ezt követően értékét átmásolja abba a felhőbe, amin dolgozni fog az alkalmazás. Hiba esetén a visszatérési értékkel jelez vissza.
 * \return
 */
int PCLViewerXModel::LoadTgtPointCloud(std::string path)
{
    if(path.empty())
        return -2;

    _tgt.reset(new PointCloudT);
    _keypoints_tgt.reset(new PointCloudT);
    _keypoints_src.reset(new PointCloudT);
    _output.reset(new PointCloudT);
    _correspondences.reset(new pcl::Correspondences);
    if(pcl::io::loadPCDFile<PointT> (path, *_tgt) == -1)
    {
        return -1;
    }
    _preprocessed_tgt.reset(new PointCloudT);
    pcl::copyPointCloud(*_tgt,*_preprocessed_tgt);
    return 0;
}


/*! \brief PCLViewerXModel::LoadSrcPointCloud
 * \param path - a betölteni kívánt fájl útvonala
 * \return - int - a betöltés eredményének az állapota. 0 helyes működés esetén, negatív érték hibás működés esetén.
 * A forrás felhő betöltéséért felelős metódus. Megkapja a megnyitni kívánt fájl útvonalát, majd a forráshoz tartozó felhőbe másolja a tartalmát a megfelelő
 * felhő betöltő metódussal. Ezt követően értékét átmásolja abba a felhőbe, amin dolgozni fog az alkalmazás. Hiba esetén -1-gyel tér vissza, helyes beolvasás
 * esetén pedig 0-val.
 */
int PCLViewerXModel::LoadSrcPointCloud(std::string path)
{
    if(path.empty())
        return -2;

    _src.reset(new PointCloudT);
    _keypoints_src.reset(new PointCloudT);
    _keypoints_tgt.reset(new PointCloudT);
    _output.reset(new PointCloudT);
    _correspondences.reset(new pcl::Correspondences);

    if(pcl::io::loadPCDFile<PointT> (path, *_src) == -1)
    {
        return -1;
    }
    _preprocessed_src.reset(new PointCloudT);
    pcl::copyPointCloud(*_src,*_preprocessed_src);
    return 0;
}


/*! \brief PCLViewerXModel::SavePreprocessedSource
* \param filename - annak a fájlnak a neve és útvonala ahova menteni kívánja a felhasználó az előfeldolgozott felhőt.
* \return - a mentésnek az állapotával tér vissza: 0 esetén helyes működés, különben negatív értékkel tér vissza.
* Az előfeldolgozott forrás felhő mentéséért felelős metódus. Megkapja a megnyitni kívánt fájl útvonalát, majd a megfelelő metódussal
* binárisan elmenti az előfeldolgozott forrás felhő tartalmát a megnyitott fájlba. A művelet sikerességét a visszaadott státusz szimbólummal jelzi.
* Üres megadott név esetén egyből visszatér -1-gyel.
*/
int PCLViewerXModel::SavePreprocessedSource(QString filename)
{
    if (filename.isEmpty ())
      return -2;

    int return_status;
    if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    {
       return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *_preprocessed_src);
    }
    else
    {
       filename.append(".pcd");
       return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *_preprocessed_src);
    }
    return return_status;
}


/*! \brief PCLViewerXModel::SavePreprocessedTarget
* \param filename - a fájl neve az útvonallal, amibe menteni szeretnénk az előfeldolgozott felhőt
* \return - az eredmény állapotát jelző INT típusú érték, aminek értéke hibamentes működés esetén 0, ellenben negatív.
**Az előfeldolgozott cél felhő mentéséért felelős metódus. Megkapja a megnyitni kívánt fájl útvonalát, majd a megfelelő metódussal
* binárisan elmenti az előfeldolgozott cél felhő tartalmát a megnyitott fájlba. A művelet sikerességét a visszaadott státusz szimbólummal jelzi.
* Üres megadott név esetén egyből visszatér -1-gyel
*/
int PCLViewerXModel::SavePreprocessedTarget(QString filename)
{
    if (filename.isEmpty ())
      return -2;

    int return_status;
    if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    {
         return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *_preprocessed_tgt);
    }
    else
     {
       filename.append(".pcd");
       return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *_preprocessed_tgt);
     }

    return return_status;

}


/*! \brief PCLViewerXModel::SaveTransformation
* \param filename : az útvonal a fájlnévvel, ahova a felhasználó menteni kívánja a transzformációs mátrixot
* \return a mentés állapotát jelző INT típusú érték, aminek helyes működés esetén az értéke 0, más különben pedig negatív.
* A transzformációs fájl mentéséért felelős metódus. Megkapja a fájl útvonalát majd szöveges formátumként átmásolja a mátrix tartalmát a fájlba.
* Üres fájl, illetve hibás megnyitás esetén a return értékkel jelzi a hibát.
*/
int
PCLViewerXModel::SaveTransformation(QString filename)
{
    if (filename.isEmpty ())
      return -2;

    int return_status;
    if (filename.endsWith (".txt", Qt::CaseInsensitive))
    {
        std::ofstream file(filename.toStdString().c_str(),std::ofstream::trunc);
          if ((return_status = file.is_open()))
          {

            file << _transform;
          }
    }
    else
     {
       filename.append(".txt");
       std::ofstream file(filename.toStdString().c_str(),std::ofstream::trunc);
         if ((return_status = file.is_open()))
         {

           file << _transform;
         }
     }
    return return_status;
}


/*! \brief PCLViewerXModel::ComputeCloudResolution
* \param cloud : a vizsgálni kívánt felhő
* \return : a kiszámított átlagos felhősűrűség
* A kapott felhő "felbontásának" kiszámításáért felelős metódus.
* A paraméterként kapott felhőnek azt a tulajdonságát számolja ki, hogy az egyes pontok átlagosan mekkora távolsággal helyezkednek el egymástól
* Ezt a műveletet úgy végzi, hogy egy fakereső algoritmussal, megkeresi a legközelebbi szomszédot, és veszi ennek négyzetes távolságát az adott ponttól
* Majd eszeket összeadja és végül átlagolja
*/
double
PCLViewerXModel::ComputeCloudResolution (const pcl::PointCloud<PointT>::ConstPtr &cloud)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud (cloud);


    for (std::size_t i = 0; i < cloud->size (); ++i)
    {
        if (! std::isfinite ((*cloud)[i].x)) //itt nem kéne mindegyiket nézni?
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances); //a felette lévő komment a tutorialban volt benne de nem igazán értem miért így működne
        if (nres == 2)
        {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}



/*! \brief PCLViewerXModel::DownSamplingBasedOnRandomSampling
* \param cloud : a vizsgált felhő
* \param downsampled_cloud : az eredményfelhő
* \param setsample : az algoritmus során használt mintaszám
* \return int: a működés állapotát jelöli. Hiba esetén -1-gyel tér vissza, helyes működés esetén 0-val.
* Csökkentett mintavételezés a megadott felhőn, eredmény kiszámítása a második paraméterként megadott felhőn. A harmadik paraméter
* a csökkentett mintavételezéshez szükséges paraméter. A Point Cloud Libraryben implementált algoritmus véletlenszerűen kiválaszt "setsample"
* darab elemet a felhőből, és ezeket adja át a másik felhőnek. Hiba esetén -1-gyel tér vissza, egyébként 0-val.
*/
int
PCLViewerXModel::DownSamplingBasedOnRandomSampling (const pcl::PointCloud<PointT>::Ptr &cloud,
                                                    pcl::PointCloud<PointT> &downsampled_cloud,double setsample)
{
    pcl::RandomSample<PointT> sample (true); // Extract removed indices
    sample.setInputCloud (cloud);
    sample.setSample (setsample);
    // Cloud
    try
    {
        sample.filter(downsampled_cloud);

    }
    catch(...)
    {
        return -1;
    }
    return 0;
}



/*! \brief PCLViewerXModel::EstimateKeypointsBasedOnISS
* \param cloud : a vizsgált felhő
* \param keypoints_cloud : a kulcspontokat tartalmaző eredményfelhő
* \param _iss_gamma_21 : az algoritmushoz használt sajátértékekkel kapcsolatos paraméter
* \param _iss_gamma_32 : az algoritmushoz használt sajátértékekkel kapcsolatos paraméter
* \param _iss_min_neighbors : az algoritmushoz használt minimális szomszédszámmal kapcsolatos paraméter
* \param _iss_SalientRadius : az algoritmushoz használt Scatter mátrix kiszámításához használt paraméter
* \param _iss_NonMaxRadius : az algoritmushoz használt lokális maximum vizsgálásához szükséges sugár paraméter paraméter
* \return : a működés helyességének állapot azonosítója
*Kulcspont detektáló módszer, ami ISS algoritmust használ. Paraméterben megadhatjuk a számítást korlátozó mennyiségekt, mint az adott pont esetén kiszámolt
* 2. és 1.  valamint a 3. és 2. saját érték hányadosának küszöbét, a minimális szomszéd számot, valamint hogy a kovariancia mátrix számítás esetén, mekkora sugárban
* számoljon az algoritmus szomszédokat. Az utolsó paraméterrel azt adhatjuk meg, hogy ha az adott pont az összes megelőző kritériumnak megfelelt, akkor mekkora legyen
* az a sugár, amiben a legnagyobb harmadik sajátértékkel kell rendelkeznie, ahhoz hogy kulcspont legyen. Ha a folyamat során hiba lép fel a program -1-es értékkel
* tér vissza. (Számítási hiba, esetleg nem talált kulcspontot).
*/
int
PCLViewerXModel::EstimateKeypointsBasedOnISS (const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZ> &keypoints_cloud,
                                              double _iss_gamma_21, double _iss_gamma_32, int _iss_min_neighbors,
                                              double _iss_SalientRadius, double _iss_NonMaxRadius)
{

    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    int nr_cores = std::thread::hardware_concurrency();
    double iss_gamma_21 =(_iss_gamma_21 ==0)?0.975:_iss_gamma_21;
    double iss_gamma_32 =(_iss_gamma_32 ==0)?0.975:_iss_gamma_32;
    double iss_min_neighbors = (_iss_min_neighbors==0)?5:_iss_min_neighbors;

    double model_resolution = ComputeCloudResolution(cloud);
    double iss_SalientRadius = (_iss_SalientRadius==0)?4*model_resolution:_iss_SalientRadius;
    double iss_NonMaxRadius = (_iss_NonMaxRadius==0)?6*model_resolution:_iss_NonMaxRadius;

    //kulcspontok kiszámolása
    pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (iss_SalientRadius);
    iss_detector.setNonMaxRadius (iss_NonMaxRadius);
    iss_detector.setThreshold21 (iss_gamma_21);
    iss_detector.setThreshold32 (iss_gamma_32);
    iss_detector.setMinNeighbors (iss_min_neighbors);
    iss_detector.setNumberOfThreads (nr_cores);
    iss_detector.setInputCloud (cloud);
    try
    {
        iss_detector.compute(keypoints_cloud);

    } catch (...)
    {
        return -1;
    }
    return 0;
}


/*! \brief PCLViewerXModel::EstimateKeypointsBasedOnSIFT
* \param cloud - bemeneti felhő
* \param keypoints_cloud - kulcspont eredmény felhő
* \param _min_scale - minimum skála érték
* \param _n_octaves - az oktávok száma
* \param _n_scales_per_octave - oktávonkénti skála szám
* \param _min_contrast -  küszöb a gauss differences értékekhez, amit átlépve kulcspont lesz az adott pont
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
*Ez a metódus felel a kulcspont kereséshez SIFT algoritmussal. A felhasználó megadhatja először azt a paramétert, amivel szeretné hogy kezdetben
* az algoritmus Voxel rács segítségével mintavételezzen, az oktávok számát (mindig új mintavételezést végez az előző paraméter kétszeresével),
* azt hogy az egyes oktávokon belül mennyi skálát számoljon (ezzel pontosíthatja az eredményt), valamint, hogy mi legyen a minimális kontraszt az egyes pontok
* és a hozzájuk kiszámolt skálák között. (Ezt Gaussok különbsége algoritmussal teszi meg)
*/
int
PCLViewerXModel::EstimateKeypointsBasedOnSIFT(const pcl::PointCloud<PointT>::Ptr &cloud,
                                              pcl::PointCloud<pcl::PointXYZ> &keypoints_cloud,
                                              const float _min_scale,
                                              const int _n_octaves ,
                                              const int _n_scales_per_octave,
                                              const float _min_contrast)
{

    float min_scale = (_min_scale==0)?0.001:_min_scale;
    int n_octaves = (_n_octaves==0)?12:_n_octaves;
    int n_scales_per_octave = (_n_scales_per_octave==0)?3:_n_scales_per_octave;
    float min_contrast = (_min_contrast==0)?0.001:_min_contrast;

    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud);

    try
    {
        sift.compute(result);
    }
    catch(...)
    {
        return -1;
    }
    copyPointCloud(result,keypoints_cloud);
    return 0;
}



/*! \brief PCLViewerXModel::EstimateKeypointsBasedOnHarrisKeypoint3D
* \param cloud - bemeneti felhő
* \param keypoints_cloud - kulcspontok tárolására szolgáló eredményfelhő
* \param _setThreshold - Intenzitás alsó küszöbe, amit át kell lépnie egy pontnak ahhoz hogy kulcspont lehessen
* \param _setNonMaxSuspression - igaz érték esetén az elöbb említett vizsgálat lefut, hamis esetén, a teljes pontfelhőt visszaadja az algoritmus kulcspontként
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
*Ez a metódus a Harris algoritmus alapú kulcspont keresésért felel. Ez az algoritmus sarkokat illetve éleket keres intenzitás számítással. A megadott paraméterek
* azt fogják jelölni, hogy mi legyen az a minimális intenzitás érték, amit kulcspontnak tekinthet az algoritmus, valamint, hogy egyáltalán vizsgálja e a felhőt
* az adott feltételekkel. Ha az utolsó paraméter hamis, akkor egyszerűen visszaadja a teljes felhőt.
*/
int
PCLViewerXModel::EstimateKeypointsBasedOnHarrisKeypoint3D(const pcl::PointCloud<PointT>::Ptr &cloud,
                                     pcl::PointCloud<pcl::PointXYZ> &keypoints_cloud,
                                     double _setThreshold, bool _setNonMaxSuspression)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    int nr_cores = std::thread::hardware_concurrency();
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_cloudI(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::HarrisKeypoint3D <PointT, pcl::PointXYZI> detector;
    detector.setNonMaxSupression (_setNonMaxSuspression);
    detector.setInputCloud (cloud);
    detector.setSearchMethod(tree);

    detector.setNumberOfThreads(nr_cores);
    detector.setThreshold((_setThreshold == 0)?0.00005:_setThreshold);
    detector.setRefine(false);

    try
    {
        detector.compute (*keypoints_cloudI);

    }
    catch(...)
    {
        return -1;
    }
    if(keypoints_cloudI->points.size() == 0)
        return  -1;

    pcl::copyPointCloud(*keypoints_cloudI, keypoints_cloud);
    return 0;

}


/*! \brief PCLViewerXModel::DownSamplingBasedOnVoxelGrid
* \param cloud - bemeneti felhő
* \param downsampled_cloud - eredményfelhő
* \param leaf - voxelrácsok oldalhosszúsága
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
*Mintavételezés Voxel ráccsal. Az első paraméterben megadott felhő alapján kiszámolja a mintavételezett felhőt a második paraméterben megadott felhőbe.
* Azt, hogy milyen mértékű legyen a ritkítás a harmadik paraméter fogja meghatározni. Ez mondja meg, hogy az algoritmusban szereplő
* voxel rácsok oldalhosszúsága mekkora legyen.
*/
int
PCLViewerXModel::DownSamplingBasedOnVoxelGrid (const pcl::PointCloud<PointT>::Ptr &cloud,
                    pcl::PointCloud<PointT> &downsampled_cloud,  double leaf)
{
      float _setleafsize = (leaf == 0)?0.01f:leaf;
      pcl::VoxelGrid<PointT> sor;
      sor.setInputCloud (cloud);
      sor.setLeafSize (_setleafsize,_setleafsize,_setleafsize);
      try
      {
          sor.filter (downsampled_cloud);
      }
      catch(...)
      {
          return -1;
      }
      return 0;
}



/*! \brief PCLViewerXModel::EstimateNormals
* \param cloud - kereső felületi felhő
* \param keypoints_cloud - kulcspontokat tartalmazó felhő
* \param normals_cloud - a normál felhő, amikbe az eredményt várjuk
* \param radiusSearch - a gömb sugara, amiben keressük a legközelebbi szomszédokat a normák meghatározásához
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
* Az adott felhőhöz normákat számító metódus. Az első paraméterben megadott felhő és a második paraméterben megadott felhő alapján
* a harmadik paraméterben megadott felhőbe számolja a normákat, felhasználva a negyedik paramétert, ami azt mondja meg, hogy a felületi normák
* számításánál mekkora sugárban vizsgálja a szomszédságát egy adott pontnak. Ezt az értéket a felhasználó a felhők átlagos sűrűségének
* szorzataként adhatja meg, ezzel könnyítve a sugár nagyságának szemléltetését. A második paraméter, akkor tér el az első paramétertől, ha a program
* kulcspontokhoz számol normákat, és ehhez egy kereső felületet határoz meg a pontosabb számolás érdekében, amit az első felhő ad meg. Ha a teljes felhőhöz
* számol kulcspontokat, akkor az első, illetve a második paraméter ugyan arra a felhőre mutat, ekkor nem veszi figyelembe az algoritmus a kereső felületet.
* Hiba esetén -1 értékkel tér vissza (Nem tudott kiszámolni egy értéket, valamiylen hiba lépett fel, esetleg a kiszámolt normákat tartalmazó felhő üres). Egyébként
* 0-val tér vissza a metódus.
*/
int
PCLViewerXModel::EstimateNormals (const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<PointT>::Ptr &keypoints_cloud,
                 pcl::PointCloud<pcl::Normal> &normals_cloud, double radiusSearch)
{
    int nr_cores = std::thread::hardware_concurrency();

    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    double _setradiussearch = (radiusSearch==0)?(10*ComputeCloudResolution(cloud)):radiusSearch;
    pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_est;
    normal_est.setInputCloud (keypoints_cloud);

    if(cloud->points.size() != keypoints_cloud->points.size())
        normal_est.setSearchSurface(cloud);

    normal_est.setSearchMethod(tree);
    normal_est.setNumberOfThreads(nr_cores);
    normal_est.setRadiusSearch ( _setradiussearch);
    try {
         normal_est.compute (normals_cloud);
    }
    catch (...)
    {
        return -1;
    }


    if(normals_cloud.points.size() == 0)
    {
        return -1;
    }

    for (unsigned long i = 0; i < normals_cloud.points.size(); i++)
    {
      if (!pcl::isFinite<pcl::Normal>(normals_cloud.points[i]))
      {
        return -2;
      }
    }
    return 0;
}


/*! \brief PCLViewerXModel::EstimateFPFH
* \param cloud - bemeneti kereső felületi felhő
* \param normals_cloud - normákat tartalmazó felhő
* \param keypoints_cloud - kulcspontokat tartalmazó felhő
* \param fpfhs_cloud - a feature leírók tárolására alkalmas flehő
* \param radiusSearch - a gömb sugara, amiben keressük a legközelebbi szomszédokat a feature leírók meghatározásához
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
*Ez a metódus az FPFH-val történő feature vektor számításért felel. Az első paraméter a kereső felhőt adja meg, a második a számításhoz szükséges
* normákat tartalmazó felhőt, a harmadik a kulcspontokat tartalmazó felhőt, amikhez feature vektorokat szeretnénk számolni. Az utolsó paraméter
* azt adja meg, hogy az egyes pontok számításánál, mekkora sugárban keressen az algoritmus szomszédokat (Túl nagy szám esetén túlságosan megegyezhetnek
* az egyes leírók, erre figyelni kell). Ezt az értéket egy szorzóval adhatjuk meg a normál számításhoz hasonlóan. A kereső sugár nagysága meg fog egyezni
* az átlagos pontfelhő sűrűség és a szorzó szorzatával. Gyakran megesik, hogy az algoritmus nem képes minden feature vektor béli értéket
* kiszámolni, és ez hibákhoz vezethet. Az ilyenek kiküszöbölésére, egy alapértelmezett értéket állítunk be a hibás adatokhoz.
* Számítási Hiba esetén a metódus -1-gyel tér vissza egyébként 0-val.
*/
int
PCLViewerXModel::EstimateFPFH (const pcl::PointCloud<PointT>::Ptr &cloud,
              const pcl::PointCloud<pcl::Normal>::Ptr &normals_cloud,
              const pcl::PointCloud<PointT>::Ptr &keypoints_cloud,
              pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_cloud,
               double radiusSearch)
{
    pcl::search::KdTree<PointT>::Ptr kdtree (new pcl::search::KdTree<PointT> ());
    double cloudresolution = ComputeCloudResolution(cloud);
    int nr_cores = std::thread::hardware_concurrency();
    double setradiussearch = (radiusSearch==0)?cloudresolution*6:radiusSearch;

    pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud (keypoints_cloud);
    fpfh_est.setSearchMethod(kdtree);
    fpfh_est.setInputNormals (normals_cloud);
    fpfh_est.setNumberOfThreads(nr_cores);
    fpfh_est.setRadiusSearch (setradiussearch);
    fpfh_est.setSearchSurface (cloud);
    fpfh_est.compute (fpfhs_cloud);


    try
    {
        fpfh_est.compute (fpfhs_cloud);

    }
    catch(...)
    {
        return -1;
    }
    for(unsigned long i = 0; i<fpfhs_cloud.points.size();i++)
    {
        for(int j = 0; j<33 ;j++)
        {
            if(!pcl_isfinite(fpfhs_cloud.points[i].histogram[j]))
            {
                fpfhs_cloud.points[i].histogram[j] = 0;
            }
        }

    }
    return 0;
}


/*! \brief PCLViewerXModel::GetFinalScore
* \param input_transformed : a transzformált forrás felhő
* \param target : a célfelhő
* \param max_range : a maximum távolság a legközelebbi szomszéd keresése során
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
*A transzformálás végeredményét mérő metódus. Megnézi minden pont esetén az első paraméterként megadott transzformált felhő esetén
* hogy az adott ponthoz milyen messze van a másik felhőben lévő legközelebbi szomszéd. Ezeket az értékeket átlagolja, így megkaptuk, hogy átlagosan
* mekkora az "elcsúszás" az egyes pontok esetén. Harmadik paraméterként egy maximum értéket állíthatunk a mért távolságnak. A visszatéréi érték pedig
* az így kiszámított érték. Hiba esetén a maximális double típusú értéket adja vissza.
*/
double
PCLViewerXModel::GetFinalScore(const pcl::PointCloud<PointT>::Ptr &input_transformed,const pcl::PointCloud<PointT>::Ptr &target,
                               double max_range)
{
    double fitness_score = 0.0;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    std::vector<int> nn_indices (1);
    tree->setInputCloud(target);
    std::vector<float> nn_dists (1);

    int nr = 0;
    for (size_t i = 0; i < input_transformed->points.size(); ++i)
    {
        if(!pcl_isfinite((*input_transformed)[i].x)){
            continue;
        }
        tree->nearestKSearch (input_transformed->points[i], 1, nn_indices, nn_dists);
        if (nn_dists[0] > max_range)
            continue;

        fitness_score += sqrt(fabs(nn_dists[0]));
        nr++;
    }
    if (nr > 0)
        return (fitness_score / nr);
    else
        return (std::numeric_limits<double>::max ());
}


/*! \brief PCLViewerXModel::FindCorrespondences
* \param feature_src - bemeneti forrás feature felhő
* \param feature_tgt - bemeneti cél feature flehő
* \param isDirectCorrespondences - bool : megmondja, hogy direkt, vagy kölcsönös párosítást szeretnénk e
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
* A párosítások megkeresésére szolgáló template metódus. Első paramétere egy T típusú forrás pont felhő (értelem szerűen valamilyen feature alapú a mi esetünkben)
* ,második paraméter egy ugyan ilyen típusú pontfelhő, ezeket akarjuk összepárosítani a PCL CorrespondenceEstimation algoritmusával, ami hasonló feature leírókat keres.
* Harmadik paraméterként megadhatjuk, hogy közvetlen legyen e a megfeleltetés vagy sem. Ez azt jelenti, hogy közvetlen esetén Az első felhő minden pontjához
* hozzárendel egy második felhőbeli pontot. Kölcsönös esetben pedig az elsőből a másodikba és a másodikból az elsőbe számolt megfeleltetések metszetét veszi.
* Ha hiba lépett fel a keresés során, vagy kevesebb mint 5 megfeleltetést számolt az algoritmus, akkor -1-es visszatérési értékkel jelzi ezt. Helyes működés esetén
* 0-val tér vissza a metódus.
*/
template<typename T>
int
PCLViewerXModel::FindCorrespondences (const typename pcl::PointCloud<T >::Ptr &feature_src,
                     const typename pcl::PointCloud<T >::Ptr &feature_tgt,bool isDirectCorrespondences
                     )
{
    CorrespondenceEstimation<T, T> est;
    est.setInputSource (feature_src);
    est.setInputTarget (feature_tgt);
    try
    {
        if(isDirectCorrespondences)
        {
            est.determineCorrespondences(*_correspondences);

        }
        else
        {
            est.determineReciprocalCorrespondences(*_correspondences);
        }

    }
    catch(...)
    {
        return -1;
    }
    if(_correspondences->size() < 5)
        return -1;

    return 0;
}


/*! \brief PCLViewerXModel::RejectBadCorrespondencesBasedOnRANSAC
* \param _setinlierthreshold - int: a belső pontok meghatározásához szükséges maximum euklédeszi távolság küszöb
* \param _setmaxit - a RANSAC iterálások száma
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
* A rossz párosítások RANSAC alapú elutasításához szükséges metódus. Első paraméterében megadhatjuk, hogy mi legyen az a maximum határérték, aminél
* kisebbnek kell lenni a párosított pontok közötti távolságnak a RANSAC transzformálást követően.
*/
int
PCLViewerXModel::RejectBadCorrespondencesBasedOnRANSAC (
                          double _setinlierthreshold ,
                                     double _setmaxit)
{
    emit SendMessage("Rejecting correspondences using CorrespondenceRejectorRANSAC...!\nThere are " + QString::number(_correspondences->size()) + " before rejection!\n");

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> corr_rej_sac;
    corr_rej_sac.setInputSource (_keypoints_src);
    corr_rej_sac.setInputTarget (_keypoints_tgt);
    double setinlierthreshold = (_setinlierthreshold == 0)?0.055:_setinlierthreshold;
    double setmaxit = (_setmaxit == 0)?3000:_setmaxit;
    corr_rej_sac.setInlierThreshold (setinlierthreshold);
    corr_rej_sac.setMaximumIterations (setmaxit);
    corr_rej_sac.setInputCorrespondences (_correspondences);
    try
    {
        corr_rej_sac.getCorrespondences (*_correspondences);
    }
    catch(...)
    {
        return -1;
    }
    emit SendMessage("Rejecting correspondences using CorrespondenceRejectorRANSAC...!\nThere are " + QString::number(_correspondences->size()) + " after rejection!\n");

    if(_correspondences->size() < 5)
        return -1;

    return 0;
}


/*! \brief PCLViewerXModel::RejectBadCorrespondencesBasedOnDistance
* \param max_distance - az int típusú maximum távolság ami megengedett a párosítások között
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
* A helytelen párosítások elutasítására szolgáló metódus távolság alapján. Azokat a párosításokat, ahol a pár két pontja között nagyobb a távolság,mint
* ami a függvény paraméterében van megadva, azokat elutasítja. Ha a maradék párosítások értéke kisebb mint 5,akkor -1 visszatérési értékkel jelzi a hibát.
* Helyes szűrésg esetén 0-val tér vissza a függvény.
*/
int
PCLViewerXModel::RejectBadCorrespondencesBasedOnDistance (double max_distance)
{
    emit SendMessage("Rejecting correspondences using CorrespondenceRejectorDistance...!\nThere are " + QString::number(_correspondences->size()) + " before rejection!\n");



    double _max_distance = (max_distance == 0)?0.05:max_distance;
    pcl::registration::CorrespondenceRejectorDistance corr_rej_dis;

    corr_rej_dis.setMaximumDistance (_max_distance);
    corr_rej_dis.setInputCorrespondences (_correspondences);
    try
    {
        corr_rej_dis.getCorrespondences (*_correspondences);
    }
    catch(...)
    {
        return -1;
    }
    emit SendMessage("There are " + QString::number(_correspondences->size()) + " after rejection!\n");
   if(_correspondences->size() < 5)
        return -1;

    return 0;
}


/*! \brief PCLViewerXModel::RejectBadCorrespondencesBasedOnOneToOne
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
* Helytelen párosítások elutasítására szolgáló metódus. Ha egy ponthoz, több célfelhőbeli pont van rendelve (vagy fordítva), akkor ezek a párosítások közül
* a legkisebb távolságút veszi az algoritmus. Ha az így kapott párosítások száma kisebb ,mint 5 akkor -1 visszatérési értékkel jelzi a hibát. Helyes eredmény
* esetén 0-val tér vissza a függvény
*/
int
PCLViewerXModel::RejectBadCorrespondencesBasedOnOneToOne()

{
    emit SendMessage("Rejecting correspondences using CorrespondenceRejectorOneToOne...!\nThere are " + QString::number(_correspondences->size()) + " before rejection!\n");
    if(_correspondences->size() < 5)
        return -1;

    pcl::CorrespondencesPtr correspondences_result_rej_one_to_one (new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
    corr_rej_one_to_one.setInputCorrespondences(_correspondences);
    try
    {
        corr_rej_one_to_one.getCorrespondences(*_correspondences);

    }
    catch(...)
    {
        return -1;
    }
    emit SendMessage("There are " + QString::number(_correspondences->size()) + " after rejection!\n");
    return 0;
}


/*! \brief PCLViewerXModel::RejectBadCorrespondencesBasedOnSurfaceNormals
* \param threshold - a maximum szög küszöb a normák elutasításához
* \param setRadiusSearchforKeypointNormals - a sugár amin belül számoljuk a normákat
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
* A metódus feladata a rossz párosítások elutasítása felületi normák használatával. A kulcspontokhoz a bemeneti felhő segítségével
* normál értékeket számol, és ha a párosítások pontjaihoz számolt normák közötti szögkülönbség radiánban nagyobb, mint az első paraméterben megadott határérték
* akkor elutasítja a párosítást. A normál számításhoz szükséges sugarat a második paraméter jelzi. Ha a végeredmény kevesebb mint 5 párosítást tartalmaz, akkor
* -1 visszatérési értékkel tér vissza a függvény, egyébként 0-val.
*/
int
PCLViewerXModel::RejectBadCorrespondencesBasedOnSurfaceNormals( double threshold, double setRadiusSearchforKeypointNormals)
{
    emit SendMessage("Rejecting correspondences using CorrespondenceRejectorSurfaceNormal...!\nThere are " + QString::number(_correspondences->size()) + " before rejection!\n");

    if(_correspondences->size() < 5)
        return -1;

    double _threshold = (threshold==0)?90:threshold;

    pcl::PointCloud<pcl::Normal>::Ptr normals__keypoints_src (new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr normals__keypoints_tgt (new pcl::PointCloud<pcl::Normal>());

    if(EstimateNormals(_preprocessed_src,_keypoints_src,*normals__keypoints_src,setRadiusSearchforKeypointNormals) != 0)
        return -1;

    if(EstimateNormals(_preprocessed_tgt,_keypoints_tgt,*normals__keypoints_tgt,setRadiusSearchforKeypointNormals) != 0)
        return -1;

    pcl::registration::CorrespondenceRejectorSurfaceNormal  corr_rej_surf_norm;
    corr_rej_surf_norm.initializeDataContainer <PointT, pcl::Normal> ();
    corr_rej_surf_norm.setInputSource <PointT> (_keypoints_src);
    corr_rej_surf_norm.setInputTarget <PointT> (_keypoints_tgt);
    corr_rej_surf_norm.setInputNormals <PointT, pcl::Normal> (normals__keypoints_src);
    corr_rej_surf_norm.setTargetNormals <PointT, pcl::Normal> (normals__keypoints_tgt);
    corr_rej_surf_norm.setInputCorrespondences (_correspondences);
    corr_rej_surf_norm.setThreshold (std::cos(_threshold));

    try
    {
        corr_rej_surf_norm.getCorrespondences (*_correspondences);
    }
    catch(...)
    {
        return -1;
    }
    emit SendMessage("There are " + QString::number(_correspondences->size()) + " after rejection!\n");
    return 0;

}


/*! \brief PCLViewerXModel::EstimatePFH
* \param cloud - keresőfelület
* \param normals_cloud - normál értékek a felületi felhőhöz
* \param keypoints_cloud - kulcspontok, amikhez számoljuk a feature leírókat
* \param pfhs_cloud - a feature leírókhoz használt felhő, amibe számoljuk a kapott eredményeket
* \param radiusSearch - a gömb sugara, amiben keressük a legközelebbi szomszédokat a feature leírók meghatározásához
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
* A metódus feladata a Feature vektorok kiszámítása PFH algoritmussal. Ez az algoritmus  Point Cloud Libraryben van implementálva. Azt ,hogy az algoritmus mekkora sugárban keressen szomszédokat, azt az utolsó paraméter
* határozza meg. Ez a paraméter egy szorzó érték, amivel a felhőhöz kiszámolt átlagos pont sűrűség lesz lesz beszorozva. A kereséshez fakereső algoritmust használ.
* Az első paraméter a kereső felületet adja meg, a második az ezekhez a pontokhoz kiszámolt normákat, a harmadik pedig a kulcspontokat, amikhez feature leírókat
* szeretnénk kiszámítani. A negyedik paraméter maga a feature vektor, amibe az érétkeket akarjuk számítani. Helytelen működés esetén a metódus -1-gyel tér vissza
* különben pedig 0-val.
*/
int PCLViewerXModel::EstimatePFH(const pcl::PointCloud<PointT>::Ptr &cloud,
                            const pcl::PointCloud<pcl::Normal>::Ptr &normals_cloud,
                            const pcl::PointCloud<PointT>::Ptr &keypoints_cloud,
                            pcl::PointCloud<pcl::PFHSignature125> &pfhs_cloud,
                             double radiusSearch)
{
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    double _setradiussearch = (radiusSearch==0)?(ComputeCloudResolution(cloud)*10):radiusSearch;

    //Elkészítjük PFH becslő osztályt és átadjuk neki a bemeneti adathalmazt, a paraméterekkel és a normákkal
    pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud (keypoints_cloud);
    pfh.setSearchSurface(cloud);
    pfh.setInputNormals (normals_cloud);
    pfh.setSearchMethod (tree);
    pfh.setRadiusSearch (_setradiussearch);
    try
    {
        pfh.compute (pfhs_cloud);
    }
    catch(...)
    {
        return -1;
    }

    for(unsigned long i = 0; i<pfhs_cloud.size();i++)
    {
        for(int j = 0; j<pfhs_cloud.points[i].descriptorSize() ;j++)
        {
            if(!pcl_isfinite(pfhs_cloud.points[i].histogram[j]))
            {
                pfhs_cloud.points[i].histogram[j] = 0;
            }
        }

    }
    return 0;
}


/*! \brief PCLViewerXModel::EstimateSHOT
* \param cloud - kereső felületi felhő
* \param normals_cloud - normákat tartalmazó bemeneti felhő
* \param keypoints_cloud - kulcspontokat tartalmazó felhő
* \param shot_cloud - a feature leírókat futás után tartalmazó eredmény felhő
* \param radiusSearch - a gömb sugara, amiben keressük a legközelebbi szomszédokat a feature leírók meghatározásához
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
* * A metódus feladata a Feature vektorok kiszámítása SHOT algoritmussal. Ez az algoritmus a Point Cloud Library-ben van implementálva. Azt ,hogy az algoritmus mekkora sugárban keressen szomszédokat, azt az utolsó paraméter
* határozza meg. Ez a paraméter egy szorzó érték, amivel a felhőhöz kiszámolt átlagos pont sűrűség lesz lesz beszorozva. A kereséshez fakereső algoritmust használ.
* Az első paraméter a kereső felületet adja meg, a második az ezekhez a pontokhoz kiszámolt normákat, a harmadik pedig a kulcspontokat, amikhez feature leírókat
* szeretnénk kiszámítani. A negyedik paraméter maga a feature vektor, amibe az érétkeket akarjuk számítani. Helytelen működés esetén a metódus -1-gyel tér vissza
* különben pedig 0-val.
*/
int
PCLViewerXModel::EstimateSHOT(const pcl::PointCloud<PointT>::Ptr &cloud,
              const pcl::PointCloud<pcl::Normal>::Ptr &normals_cloud,
              const pcl::PointCloud<PointT>::Ptr &keypoints_cloud,
              pcl::PointCloud<pcl::SHOT352> &shot_cloud,
              double radiusSearch)
{
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    double _setradiussearch = (radiusSearch==0)?(10*ComputeCloudResolution(cloud)):radiusSearch;
    int nr_cores = std::thread::hardware_concurrency();

    pcl::SHOTEstimationOMP<PointT, pcl::Normal, pcl::SHOT352> shotEstimation;
    shotEstimation.setInputCloud(keypoints_cloud);
    shotEstimation.setInputNormals(normals_cloud);
    shotEstimation.setSearchSurface(cloud);
    shotEstimation.setRadiusSearch(_setradiussearch);
    shotEstimation.setSearchMethod(tree);
    shotEstimation.setNumberOfThreads(nr_cores);
    try
    {
        shotEstimation.compute(shot_cloud);

    }
    catch(...)
    {
        return -1;
    }

    for(unsigned long i = 0; i<shot_cloud.size();i++)
    {
        for(int j = 0; j<shot_cloud.points[i].descriptorSize() ;j++)
        {
            if(!pcl_isfinite(shot_cloud.points[i].descriptor[j]))
            {
                shot_cloud.points[i].descriptor[j] = 0;
            }
        }

    }
    return 0;
}



/*! \brief PCLViewerXModel::ICPRegistration
* \param icpmaxdes - a maximum távolság a megfeleltetések között
* \param setransacthreshold - az elutasítóhoz használt maximum küszöbérték a belső pontok meghatározásához
* \param icpmaxit - az ICP maximum iterálási száma
* \param setransacit - az elutasíthoz használt maximum RANSAC iterálások száma
* \param setEuclideanFitnessEpsilon - az ICP-hez használt konvergálási küszöb
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
*Ez a metódus végzi a regisztrációt Iterative Closest Point algoritmus alapján, ami a PCL-ben van implementálva.
* Ez az algoritmus addig iterál és legközelebbi pontokat keres, majd ebbe transzformál, amíg egy megállási kritériumba nem ér az algoritmus.
* A maximum távolságot az egyes párosítani kívánt pontok között az első paraméter adja meg, a második paraméter a RANSAC párosítás elutasítóhoz használt
* határérték, a harmadik paraméter az ICP maximum iterálási száma (egy megállási feltétel), a negyedik algoritmus a RANSAC elutasító maximum iterálási száma,
* az utolsó paraméter pedig az a határtérték, amit elérve terminálhat az ICP algoritmus. Ez a határérték az illesztés pontosságát adja meg (Euklédeszi távolságok átlagával az egyes pontokhoz).
* Hiba esetén -1-gyel tér vissz a függvény , helyes működés esetén pedig a kiszámolt végleges transzformálás hibametrikájával tér vissza.
*/
double
PCLViewerXModel::ICPRegistration( double icpmaxdes, double setransacthreshold,double icpmaxit,double setransacit, double setEuclideanFitnessEpsilon)
{
    _correspondences.reset(new pcl::Correspondences);
    _keypoints_src.reset(new PointCloudT);
    _keypoints_tgt.reset(new PointCloudT);
    _output.reset(new PointCloudT);
    _transform.setIdentity();

    PointCloudT::Ptr  result (new PointCloudT);

    if(_preprocessed_src->points.size() == 0 || _preprocessed_tgt->points.size() == 0)
        return -1;

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(_preprocessed_src);
    icp.setInputTarget(_preprocessed_tgt);
    icp.setRANSACOutlierRejectionThreshold((setransacthreshold)?0.07:setransacthreshold); //0.07
    icp.setRANSACIterations((setransacit==0)?9999:setransacit); //9999
    icp.setMaximumIterations((icpmaxit==0)?9999:icpmaxit);//9999
    icp.setMaxCorrespondenceDistance((icpmaxdes==0)?0.07:icpmaxdes); //0.07
    icp.setEuclideanFitnessEpsilon((setEuclideanFitnessEpsilon==0)?0.000005:setEuclideanFitnessEpsilon); //0.000005
     try
     {
          icp.align(*result);
     }
     catch(...)
     {
         return -2;
     }
     _transform =  icp.getFinalTransformation() ;
     transformPointCloud (*_src, *_output, _transform );
     return GetFinalScore(_output,_tgt);

}


/*! \brief PCLViewerXModel::RANSACRegistration
* \param _leafsizefordownsample - a voxel rácshoz szükséges paraméter
* \param _setradiussearchfornormals - a normálszámításhoz szükséges paraméter
* \param _setradiussearchforfeatures - a feature számításhoz szükséges paraméter
* \param _setmaxit - a RANSAC iterálások száma
* \param _setnumberofsamples - a párosítások kiválasztásához használt mintaszám
* \param _setCorrespondenceRandomness - a feature leírók párosításához használt véletlenszerű kiválasztás növelésére szolgáló paraméter
* \param _setsimilaritythreshold - az elő-elutasító algoritmushoz használt hasonlósági paraméter
* \param _setInlierFraction - a belső pontok aránya a forrásban, ahhoz hogy elfogadjon az algoritmus egy hipotézist
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
* A metódus a RANSAC algoritmust használja a Regisztrációhoz, kiegészítve egy megelőző elutasító algoritmussal.
* Kezdetben "kulcspont" detektáláshoz Voxel rács segítségével megritkítja a felhőt, majd normákat számol a bemeneti felhőkhöz a megfelelő metódus segítségével.
* Ezt követően FPFH-val feature leírókat számol, majd a kapott eredményeket a RANSAC Regisztráció bemeneti értékeihez adja. Az első paraméter a Voxel mintavételezéshez
* használt paramétert adja meg, a második a normák számításához használt sugár paraméter értékét, a harmadik a feature leírókhoz használt sugár paraméter értékét
* a negyedik paraméter a RANSAC iterálások maximális száma, az ötödik megadja az egyes iterálások között vett mintavételi pontok számát. A hatodik paraméter a _setCorrespondenceRandomness
* amivel megadható, hogy az egyes minták esetén, az adott feature leíróhoz legjobban hasonlító feature leírók közül hány legjobbat válasszon, amelyek közül az algoritmus majd véletlenszerűen választ.
* A _setsimilaritythreshold megadja, hogy mennyire legyen elutasító az előzetes elutasítás során az algoritmus, vagyis, hogy mennyire kell hogy hasonlók
* legyenek az egyes feature vektorok, ezt egy 0 és 1 közötti érték adja meg ahol 0
* esetén ez a funkció ki van kapcsolva, 1 esetén pedig teljesen elutasító. A _setInlierFraction, azt határozza meg, hogy mekkora legyen a belső pontok aránya a teljes bemeneti felhőhöz képest, ahhoz hogy az
* algoritmus egy adott hipotézist elfogadjon. A _setInlierThreshold paraméter megmondja, hogy az egyes RANSAC iterálások során, mekkora legyen a párosított elemek közötti távolság a térben a transzformációkat követően,
* ezzel egy hibametrikát határozva meg. Ha a határérték alatt van a számolt távolság, akkor belső veszi az algoritmus. Hibás számítás esetén a metódus
* -1-gyel tér vissza, helyes működés esetén pedig a kiszámolt transzformálás hibametrikájával.
*/
double PCLViewerXModel::RANSACRegistration(double _leafsizefordownsample,double _setradiussearchfornormals,
                                           double _setradiussearchforfeatures,int _setmaxit,int _setnumberofsamples,
                                           int _setCorrespondenceRandomness,double _setsimilaritythreshold,
                                           double _setInlierFraction, double _setInlierThreshold)
{
    //reseteljük a pointereket, hogy az eddigi regisztrálások eredményét frissítsük
    _correspondences.reset(new pcl::Correspondences);
    _keypoints_src.reset(new PointCloudT);
    _keypoints_tgt.reset(new PointCloudT);
    _output.reset(new PointCloudT);
    _transform.setIdentity();


    //ellenőrizzük, hogy a bemeneti felhők jók-e
    if(_preprocessed_src->points.size() == 0 || _preprocessed_tgt->points.size() == 0)
        return -1;

    emit SendMessage("Registration based on RANSAC has started..!");
    PointCloudT::Ptr  result (new PointCloudT);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features (new pcl::PointCloud<pcl::FPFHSignature33>),target_features (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::Normal>::Ptr source_normals (new pcl::PointCloud<pcl::Normal>),target_normals (new pcl::PointCloud<pcl::Normal>);

    //Ha a felhasználó 0-n hagyta a paramétereket, akkor az alapértelmezett értékek lesznek beállítva
    double leaf = (_leafsizefordownsample == 0)?0.005f:_leafsizefordownsample;
    double radiussearch = (_setradiussearchfornormals == 0)?0.2:_setradiussearchfornormals;
    double radiussearchforfeatures= (_setradiussearchforfeatures == 0)?0.25f:_setradiussearchforfeatures;
    double maxit= (_setmaxit == 0)?5000:_setmaxit;
    double numberofsamples= (_setnumberofsamples == 0)?3:_setnumberofsamples;
    double randomness= (_setCorrespondenceRandomness == 0)?5:_setCorrespondenceRandomness;
    double setsimilaritythreshold = (_setsimilaritythreshold == 0)?0.9:_setsimilaritythreshold;
    double setInlierThreshold = (_setInlierThreshold == 0)?0.05:_setInlierThreshold;
    double inlierfraction= (_setInlierFraction == 0)?0.25f:_setInlierFraction;

    emit SendMessage("Ransac: downsampling to estimate keypoints...\n");
    pcl::VoxelGrid<PointT> grid;
    if(DownSamplingBasedOnVoxelGrid(_preprocessed_src,*_keypoints_src,leaf) != 0)
        return -2;

    if(DownSamplingBasedOnVoxelGrid(_preprocessed_tgt,*_keypoints_tgt,leaf) != 0)
        return -2;

    emit SendMessage("Ransac: Computed " + QString::number(_keypoints_src->points.size()) + " keypoints in source...\n" +
                     "Ransac: Computed " + QString::number(_keypoints_tgt->points.size()) + " keypoints in target...\n" +
                     "Ransac: Estimating normals...\n");

    if(EstimateNormals(_preprocessed_src,_preprocessed_src,*source_normals,radiussearch) != 0)
        return -3;

    if(EstimateNormals(_preprocessed_tgt,_preprocessed_tgt,*target_normals,radiussearch) != 0)
        return -3;

    emit SendMessage("RANSAC: estimating normals finished..!\nFeature based registration: Computed " + QString::number(source_normals->points.size()) + "point normal for source!\n" +
                         "Feature based registration: Computed " + QString::number(target_normals->points.size()) + "point normal for target!\nRansac: Estimating features...\n");


    if(EstimateFPFH(_preprocessed_src,source_normals,_keypoints_src, *source_features,radiussearchforfeatures) != 0)
        return -4;
    if(EstimateFPFH(_preprocessed_tgt,target_normals,_keypoints_tgt,*target_features,radiussearchforfeatures) != 0)
        return -4;

    if(source_features->size() < 5 || target_features->size() < 5)
        return -4;

    emit SendMessage("Ransac: Starting registration...\n");
    pcl::SampleConsensusPrerejective<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> align;
    align.setInputSource (_keypoints_src);
    align.setSourceFeatures (source_features);
    align.setInputTarget (_keypoints_tgt);
    align.setTargetFeatures (target_features);
    align.setRANSACOutlierRejectionThreshold(setInlierThreshold);
    align.setMaximumIterations (maxit);
    align.setNumberOfSamples (numberofsamples);
    align.setCorrespondenceRandomness (randomness);
    align.setSimilarityThreshold (setsimilaritythreshold);
    align.setInlierFraction (inlierfraction);
    try
    {
        align.align(*result);
    }
    catch(...)
    {
        return -5;
    }
    if(align.hasConverged())
    {
        _transform = align.getFinalTransformation();
        transformPointCloud (*_src, *_output, _transform );
        return GetFinalScore(_output,_tgt);
    }
    else
        return -6;

}


/*! \brief PCLViewerXModel::OutliersRemovalBasedOnStatistical
* \param cloud - bementi felhő
* \param filtered_cloud - eredmény felhő
* \param _setmeanK - vizsgált szomszédok száma
* \param _setStddevMulThresh - a szorzóérték a küszöb meghatározásához
* \param setNegative - ellentétes működés beállításához szükséges paraméter
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
*Statisztikailag kiálló adatok eltávolítására felelős metódus. Ehhez a PCL-nek a StatisticalOutlierRemoval osztályát használja a metódus.
* Az első paraméter a vizsgált felhő, a második paraméter pedig az eredmény felhő. A _setmeanK paraméter megadja, hogy az egyes pontokhoz mért átlagos
* távolságok méréséhez mennyi szomszédot keressen. A második paraméter azt adja meg, hogy az előző algoritmus alapján kiszámolt értékek átlagát és szórását véve
* mekkora legyen az a küszöb ami esetén elutasítsa az algoritmus az adott pontot (átlag + szórás * _setStddevMulThres). Ha ennél a küszöbnél nagyobb az egyes pontok szomszédsági száma, akkor
* az adott pont el lesz utasítva, ha pedig nem, akkor nem. Az utolsó paraméter fordított működést eredményezi, tehát a kiszámolt értékek negáltját veszi
* a felhőre nézve. (Minden olyan értéket amit kiállónak minősített elfogadja, a többit elutasítja). Helyes működés esetén a program 0-val tér vissza,
* helytelen működés esetén pedig -1-gyel jelzi a hibát.
*/
int
PCLViewerXModel::OutliersRemovalBasedOnStatistical(
        pcl::PointCloud<PointT>::Ptr &cloud,  pcl::PointCloud<PointT> &filtered_cloud,
         int _setmeanK, double _setStddevMulThresh, bool setNegative)
{
    emit SendMessage("Remove outliers using StatisticalOutlierRemoval..");
    int setmeanK = (_setmeanK==0)?15:_setmeanK;
    double setStddevMulThresh = (_setStddevMulThresh==0)?0.05:_setStddevMulThresh;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (setmeanK);
    sor.setStddevMulThresh (setStddevMulThresh);
    sor.setNegative (setNegative);
    try
    {
        sor.filter (filtered_cloud);
    }
    catch(...)
    {
        return -1;
    }
    return 0;
}


/*! \brief PCLViewerXModel::OutliersRemovalBasedOnRadius
* \param cloud - bemeneti felhő
* \param filtered_cloud - eredményfelhő
* \param _setMinNeighborsInRadius - minimális szomszédok száma a sugárban
* \param _setRadiusSearch - a sugár nagysága
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
* Ez a metódus felelős a kiálló részek eltávolításáért sugár alapján. Az első paraméter megadja a vizsgált felhőt, a második az eredmény felhőt,
* ahova ki kell számítani az eredményt, a harmadik paraméter jelöli a minimális szomszédsági számot, aminek meg kell lennie a negyedik paraméter
* által megadott sugárban, ahhoz hogy egy pontot ne távolítson el az algoritmus. Ez az algoritmus a PCL könyvtárban van implementálva. Helyes működés esetén
* a program 0-val tér vissza, egyébként pedig -1-gyel.
*/
int PCLViewerXModel::OutliersRemovalBasedOnRadius(
        pcl::PointCloud<PointT>::Ptr &cloud,  pcl::PointCloud<PointT> &filtered_cloud,
       int _setMinNeighborsInRadius, double _setRadiusSearch)
{
    emit SendMessage("Remove outliers using RadiusOutlierRemoval...");
    int setMinNeighborsInRadius = (_setMinNeighborsInRadius==0)?15:_setMinNeighborsInRadius;
    double setRadiusSearch = (_setRadiusSearch==0)?0.05:_setRadiusSearch;

    //objektum létrehozása az eltávolításhoz, paraméterei beállítása
    pcl::RadiusOutlierRemoval<PointT> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(setRadiusSearch);
    outrem.setMinNeighborsInRadius (setMinNeighborsInRadius);
    try
    {
        outrem.filter (filtered_cloud);
    }
    catch(...)
    {
        return -1;
    }
    return 0;
}


/*! \brief PCLViewerXModel::SmoothingBasedOnMovingLeastSquares
* \param cloud - bemeneti felhő
* \param filtered_cloud - eredményfelhő
* \param _setSearchRadius - sugár a simításhoz
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
*Az MLS algoritmus alapján történő simítást végző metódus, ami a PCL-ben implementált MovingLeastSquares osztályt használjaa simításhoz.
* Azt, hogy a súlypont számításokhoz, az egyes pontokhoz mekkora sugarat vegyen figyelembe az algoritmus, azt a _setSearchRadius
* paraméter adja meg. Az első paraméter a bemeneti felhő, a második pedig az a felhő, amibe eltárolni kívánjuk az eredményt. Helyes számítás esetén
* 0-val tér vissza a program, helytelen számítás esetén pedig -1-gyel.
*/
int
PCLViewerXModel::SmoothingBasedOnMovingLeastSquares( pcl::PointCloud<PointT>::Ptr &cloud,  pcl::PointCloud<PointT> &filtered_cloud,
                                            double _setSearchRadius)
{
    emit SendMessage("Smoothing using MLS...!\n");
    PointCloudT::Ptr result(new PointCloudT);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); //keresőfa algoritmus használata
    double setSearchRadius = (_setSearchRadius==0)?0.03:_setSearchRadius;
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setComputeNormals (false); //normák kiszámítását kikapcsoljuk, a gyorsabb működés érdekében
    // paraméterek beállítása
    mls.setInputCloud (cloud);
    mls.setPolynomialFit(false); //kikapcsoljuk, azt hogy polinom közelítéssel számoljon az algoritmus a gyorsabb működés érdekében
    mls.setSearchMethod (tree);
    mls.setSearchRadius (setSearchRadius);

    // Reconstruct
    try
    {
        mls.process (*result);

    }
    catch(...)
    {
        return -1;
    }

    copyPointCloud(*result,filtered_cloud);
    return 0;
}



/*!
 * \brief PCLViewerXModel::getSrc
 * \return - eredeti forrás felhőt adja vissza
 */
PointCloudT::Ptr PCLViewerXModel::getSrc() const
{
    return _src;
}

/*!
 * \brief PCLViewerXModel::getTgt
 * \return eredeti célfelhő
 */
PointCloudT::Ptr PCLViewerXModel::getTgt() const
{
    return _tgt;
}

/*!
 * \brief PCLViewerXModel::getPreprocessed_src
 * \return előfeldolgozott forrásfelhő
 */
PointCloudT::Ptr PCLViewerXModel::getPreprocessed_src() const
{
    return _preprocessed_src;
}

/*!
 * \brief PCLViewerXModel::getPreprocessed_tgt
 * \return előfeldolgozott célfelhő
 */
PointCloudT::Ptr PCLViewerXModel::getPreprocessed_tgt() const
{
    return _preprocessed_tgt;
}

/*!
 * \brief PCLViewerXModel::getKeypoints_src
 * \return kulcspontok a forráshoz
 */
PointCloudT::Ptr PCLViewerXModel::getKeypoints_src() const
{
    return _keypoints_src;
}

/*!
 * \brief PCLViewerXModel::getKeypoints_tgt
 * \return kulcspontok a célfelhőhöz
 */
PointCloudT::Ptr PCLViewerXModel::getKeypoints_tgt() const
{
    return _keypoints_tgt;
}

/*!
 * \brief PCLViewerXModel::getGood_correspondences
 * \return a párosítások
 */
pcl::CorrespondencesPtr PCLViewerXModel::getGood_correspondences() const
{
    return _correspondences;
}

/*!
 * \brief PCLViewerXModel::getOutput
 * \return transzformált forrásfelhőt adja vissza
 */
PointCloudT::Ptr PCLViewerXModel::getOutput() const
{
    return _output;
}


/*! \brief PCLViewerXModel::ComputeCloudDiameter
* \param cloud - bemeneti felhő
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
* A bemeneti felhő átmérőjét visszaadó metódus. Visszatérési értéke a kiszámolt átmérő.
*/
double PCLViewerXModel::ComputeCloudDiameter(const pcl::PointCloud<PointT>::Ptr &cloud)
{
    if(cloud == nullptr || cloud->points.size() == 0 )
    {
        return -1;
    }
    PointT _min;
    PointT _max;
    pcl::getMinMax3D(*cloud,_min,_max); //minimum, maximum pontok számolása a 3 dimenziós térben.
    double distance = sqrt(pow((_min.x - _max.x),2)  + pow((_min.y - _max.y),2)  + pow((_min.z - _max.z),2));//ezek közötti távolság kiszámítása
    return distance;
}


/*! A feature alapú regisztrálást végző metódus. Ami a kapott paraméterek alapján meghívja a megfelelő függvényeket és hiba esetén jelez. Utolsó lépésként
* a kiszámolt transzformációt elvégzi a forrás felhőn, és hibamentes működés esetén pedig az így kapott regisztráció kiszámított hibametrikájával tér vissza.
* Az egyes szekciókban történő hibák esetére külön-külön hiba kódokkal tér vissza.
* \params: az egyes műveletekhe szükséges paraméterek
* \return : a működés helyességének állapotazonosítója (0 helyes működés, negatív érték helytelen működés)
*/
double
PCLViewerXModel::FeatureBasedRegistration(bool _HarrisKeypoints, bool _ISSKeypoints,// bool _SIFTKeypoints,
                                      bool _FPFH, bool _PFH,// bool //_SHOT,
                                      bool IsDirectCorrespondences,
                                      bool _RANSACREJ, bool _distanceREJ, bool _onetooneREJ, bool _surfacenormalsREJ,
                                      bool _SVDTRANS, //bool //_LMTRANS,
 double _Harris_ThresholdS, double _Harris_ThresholdT, bool _Harris_setNonMaxSupS, bool _Harris_setNonMaxSupT,//HARRIS
 double _ISS_Gamma21S, double _ISS_Gamma21T, double _ISS_Gamma32S, double _ISS_Gamma32T, int _ISS_MINNS, int _ISS_MINNT,//ISS
 double _ISS_SalientRadS, double _ISS_SalientRadT, double _ISS_SetNonSupS, double _ISS_SetNonSupT,
 //SIFT
 int _SIFT_noctavesS, int _SIFT_noctavesT, int _SIFT_nscalesperoctavesS, int _SIFT_nscalesperoctavesT, double _SIFT_minscaleS, double _SIFT_minscaleT, double _SIFT_mincontrastS, double _SIFT_mincontrastT,
 //features
 double _radiusSearchForFeatures, double _radiusSearchForNormals,
 //rejection
 double _RANSACREJ_setInlierThreshold, double _RANSACREJ_setMaximumIterations, double _distanceREJ_setMaxDistance, double _surfacenormalsREJ_setThresHold, double _surfacenormalsREJ_setRadiusSearch
)
{
    emit SendMessage("Feature based registration has started..!\n");
    if(_preprocessed_src == nullptr || _preprocessed_src->points.size() == 0 ||_preprocessed_tgt == nullptr || _preprocessed_tgt->points.size()==0)
        return -7;

    //párosítások reinicializálása
    _correspondences.reset(new pcl::Correspondences);
    _keypoints_src.reset(new PointCloudT);
    _keypoints_tgt.reset(new PointCloudT);
    _transform.setIdentity();


    //Not a number pontok eltávolítása a felhőből
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices ());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src (new pcl::PointCloud<pcl::FPFHSignature33>),fpfhs_tgt (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_src ( new pcl::PointCloud<pcl::PFHSignature125>), pfhs_tgt(new pcl::PointCloud<pcl::PFHSignature125>);
    pcl::PointCloud<pcl::SHOT352>::Ptr shot_src(new pcl::PointCloud<pcl::SHOT352>),shot_tgt(new pcl::PointCloud<pcl::SHOT352>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>),normals_tgt (new pcl::PointCloud<pcl::Normal>);


    pcl::removeNaNFromPointCloud(*_preprocessed_src,*_preprocessed_src,inliers->indices);
    pcl::removeNaNFromPointCloud(*_preprocessed_tgt,*_preprocessed_tgt,inliers2->indices);

    emit SendMessage("Feature based registration:  Source has " + QString::number(_preprocessed_src->points.size()) + "points!\n"
                     + "Feature based registration:  Target has " + QString::number(_preprocessed_tgt->points.size()) + "points!\n\nEstimating normals...\n");


    //Normák kiszámítása, hibás érték visszaadása esetén -1-gyel visszatérés
    if(EstimateNormals(_preprocessed_src,_preprocessed_src,*normals_src, _radiusSearchForNormals) != 0)
        return -1;

    if(EstimateNormals(_preprocessed_tgt,_preprocessed_tgt,*normals_tgt, _radiusSearchForNormals) != 0)
        return -1;

    //Kulcspont detektálás megkezdése
    if(_HarrisKeypoints)
    {
        emit SendMessage("Feature based registration: estimating keypoints has started (HarrisKeyPoint3D)!\n");

       //Ha a Harris kulcspont detektáló volt bejelölve, akkor ez az ág fut le, hibás kiszámítás esetén -2-vel tér vissza a függvény
       if(EstimateKeypointsBasedOnHarrisKeypoint3D(_preprocessed_src,*_keypoints_src,_Harris_ThresholdS, _Harris_setNonMaxSupS) != 0)
           return -2;


       if(EstimateKeypointsBasedOnHarrisKeypoint3D(_preprocessed_tgt,*_keypoints_tgt,_Harris_ThresholdT, _Harris_setNonMaxSupT) != 0)
           return -2;

    }
    else if(_ISSKeypoints)
    {
        emit SendMessage("Feature based registration: estimating keypoints has started (ISS)!\n");
        //Ha az ISS kulcspont detektáló volt bejelölve, akkor ez az ág fut le, hibás kiszámítás esetén -2-vel tér vissza a függvény
        if(EstimateKeypointsBasedOnISS(_preprocessed_src,*_keypoints_src,_ISS_Gamma21S,_ISS_Gamma32S,  _ISS_MINNS, _ISS_SalientRadS, _ISS_SetNonSupS) != 0)
            return -2;

        if(EstimateKeypointsBasedOnISS(_preprocessed_tgt,*_keypoints_tgt,_ISS_Gamma21T, _ISS_Gamma32T,  _ISS_MINNT, _ISS_SalientRadT, _ISS_SetNonSupT) != 0)
            return -2;
    }
    else
    {
        emit SendMessage("Feature based registration: estimating keypoints has started (SIFT)!\n");
        //Ha a SIFT kulcspont detektáló volt bejelölve, akkor ez az ág fut le, hibás kiszámítás esetén -2-vel tér vissza a függvény
        if(EstimateKeypointsBasedOnSIFT(_preprocessed_src,*_keypoints_src,_SIFT_minscaleS, _SIFT_noctavesS, _SIFT_nscalesperoctavesS,_SIFT_mincontrastS) != 0)
            return -2;

        if(EstimateKeypointsBasedOnSIFT(_preprocessed_tgt,*_keypoints_tgt,_SIFT_minscaleT, _SIFT_noctavesT, _SIFT_nscalesperoctavesT,_SIFT_mincontrastT) != 0)
            return -2;

    }

    if(_keypoints_src->points.size()==0 || _keypoints_tgt->points.size()==0)
    {
        return -2;
    }

    //Feature leírók kiszámításának megkezdése
    emit SendMessage("Feature based registration: Computed " + QString::number(_keypoints_src->points.size()) + "keypoints in source!\n" +
                     "Feature based registration: Computed " + QString::number(_keypoints_tgt->points.size()) + "keypoints in target!\n\nFeature based registration: estimating feature descriptor starting!\n\n");

    if(_FPFH)
    {
        //Ha az FPFH volt kiválasztva, akkor ez az ág fut le. Hibás érték esetén visszatérünk -3-mal.
        if(EstimateFPFH(_preprocessed_src,normals_src,_keypoints_src, *fpfhs_src,_radiusSearchForFeatures) != 0)
            return -3;
        if(EstimateFPFH(_preprocessed_tgt,normals_tgt,_keypoints_tgt,*fpfhs_tgt,_radiusSearchForFeatures) != 0)
            return -3;
        emit SendMessage("Feature based registration: estimating feature descriptor finished!\n\nFeature based registration: estimating correspondences has started!\n");

        if(FindCorrespondences<pcl::FPFHSignature33> (fpfhs_src, fpfhs_tgt, IsDirectCorrespondences) != 0)//Párosítások keresése az FPFH feature leíróhoz, hibás visszatérési érték esetén -4-gyel tér vissza a függvény.
            return -4;

    }
    else if(_PFH)
    {
        //Ha a PFH volt kiválasztva, akkor ez az ág fut le. Hibás érték esetén visszatérünk -3-mal.
        if(EstimatePFH(_preprocessed_src,normals_src,_keypoints_src, *pfhs_src,_radiusSearchForFeatures) != 0)
            return -3;
        if(EstimatePFH(_preprocessed_tgt,normals_tgt,_keypoints_tgt,*pfhs_tgt,_radiusSearchForFeatures) != 0)
            return -3;

        emit SendMessage("Feature based registration: estimating feature descriptor finished!\n\nFeature based registration: estimating correspondences has started!\n");
        if(FindCorrespondences<pcl::PFHSignature125> (pfhs_src, pfhs_tgt,IsDirectCorrespondences)!=0)//Párosítások keresése az PFH feature leíróhoz, hibás visszatérési érték esetén -4-gyel tér vissza a függvény.
            return -4;

    }
    else
    {
        //Ha a SHOT volt kiválasztva, akkor ez az ág fut le. Hibás érték esetén visszatérünk -3-mal.
        if(EstimateSHOT(_preprocessed_src,normals_src,_keypoints_src, *shot_src,_radiusSearchForFeatures) != 0)
            return -3;
        if(EstimateSHOT(_preprocessed_tgt,normals_tgt,_keypoints_tgt,*shot_tgt,_radiusSearchForFeatures) != 0)
            return -3;

        emit SendMessage("Feature based registration: estimating feature descriptor finished!\n\nFeature based registration: estimating correspondences has started!\n");
        if(FindCorrespondences<pcl::SHOT352 > (shot_src,shot_tgt,IsDirectCorrespondences) != 0)//Párosítások keresése az SHOT feature leíróhoz, hibás visszatérési érték esetén -4-gyel tér vissza a függvény.
            return -4;
    }
    emit SendMessage("Feature based registration: Computed " + QString::number(_correspondences->size()) + " correspondences!\nFeature based registration: rejection correspondences has started!\n");

    //Rossz párosítások elutasításának megkezdése

    if(_RANSACREJ)
    {
        //Ha a RANSAC lett kiválasztva, akkor ez az ág fut le, rossz visszatérési érték esetén visszatérünk -5-tel.
        if(RejectBadCorrespondencesBasedOnRANSAC (_RANSACREJ_setInlierThreshold   ,_RANSACREJ_setMaximumIterations)!=0)
            return -5;

    }

    if(_distanceREJ)
    {
        //Ha a távolság alapú algoritmus lett kiválasztva, akkor ez az ág fut le, rossz visszatérési érték esetén visszatérünk -5-tel.
        if(RejectBadCorrespondencesBasedOnDistance(_distanceREJ_setMaxDistance) != 0)
            return -5;
    }

    if(_onetooneREJ)
    {
        //Ha az "Egyhez egy" alapú algoritmus lett kiválasztva, akkor ez az ág fut le, rossz visszatérési érték esetén visszatérünk -5-tel.
        if(RejectBadCorrespondencesBasedOnOneToOne() != 0)
            return -5;
    }

    if(_surfacenormalsREJ)
    {
        //Ha a rossz párosítások felületi normák segítségével lett kiválasztva, akkor ez az ág fut le, rossz visszatérési érték esetén visszatérünk -5-tel.

        if(RejectBadCorrespondencesBasedOnSurfaceNormals(_surfacenormalsREJ_setThresHold, _surfacenormalsREJ_setRadiusSearch) != 0)
            return -5;
    }

    //megnézzük, hogy a párosítások száma nagyobb e mint 4 (a minimum követelmény)
    if(_correspondences->size() >=5)
    {
        if(_SVDTRANS)
        {
            //Ha az SVD alapú transzformáció lett kiválasztva akkor ez az ág fog lefutni. Hiba esetén -6tal tér vissza a függvény.
             TransformationEstimationSVD<PointT, PointT> trans_est;
             emit SendMessage("Estimate rigid transformation SVD...\n");
             try
             {
                trans_est.estimateRigidTransformation (*_keypoints_src, *_keypoints_tgt, *_correspondences, _transform);
             }
             catch(...)
             {
                 return -6;
             }
        }
        else
        {
            //Ha az LM alapú transzformáció lett kiválasztva akkor ez az ág fog lefutni. Hiba esetén -6tal tér vissza a függvény.
              const pcl::registration::TransformationEstimationLM<PointT, PointT, float> trans_est_lm_float;
              emit SendMessage("Estimate rigid transformation LM...\n");
              try
              {
                  trans_est_lm_float.estimateRigidTransformation (*_keypoints_src, *_keypoints_tgt, *_correspondences, _transform);
              }
              catch(...)
              {
                  return -6;
              }
        }
        transformPointCloud (*_src, *_output, _transform );        //Hiba nélküli futás esetén transzformáljuk a felhőt
        emit SendMessage("Feature based registration: estimating transformation finished!\n");
        return GetFinalScore(_output,_tgt); //és visszatérünk a transzformálás eredményével
    }
    else {
        //Ha kevesebb, mint 5 párosítás volt, akkor hiba lép fel és -6-os értékkel visszatérünk.
        emit SendMessage("Feature based registration: Transformation failed. Error: You need at least 5 correspondences to estimating  transformation!\n");
        return -6;

    }
}

/*!
 * \brief PCLViewerXModel::Preprocessing
 * \param downsampling : Előfeldolgozást végezzen e a függvény
 * \param downsampling_source : A forrás felhőn végezzen-e az előfeldolgozást
 * \param downsampling_target : A célfelhőn végezzen e előfeldolgozást
 * \param voxeldownsampling : Voxel módszert használjon e előfeldolgozáshoz
 * \param outliersremoval : kiugró részeket eltávolítsa e az előfeldolgozás
 * \param outliersremoval_source : kiugró részek eltávolítása a forráson
 * \param outliersremoval_target : kiugró részek eltávolítása a célon
 * \param removal_statistical : statisztikailag kiugró részek eltávolítása
 * \param smoothing : simítást végezzen e az előfeldolgozás
 * \param smoothing_source : simítson e a forrás felhőn
 * \param smoothing_target : simítson e a célfelhőn
 * \param randomsampling_setsample_source : random mintavételezéshez szükséges paraméter a forráshoz
 * \param randomsampling_setsample_target : random mintavételezéshez szükséges paraméter a célhoz
 * \param voxel_leaf_source : voxel mintavételezéshez szükséges paraméter a forráshoz
 * \param voxel_leaf_target : voxel mintavételezéshez szükséges paraméter acélhoz
 * \param statistical_setmeanK_source : statisztikailag kiugró részek eltávolításához szükséges paraméter a forráshoz
 * \param statistical_setStddevMulThres_source : statisztikailag kiugró részek eltávolításához szükséges paraméter a forráshoz
 * \param setNegative_source : statisztikailag kiugró részek eltávolításához szükséges paraméter a forráshoz
 * \param statistical_setmeanK_target : statisztikailag kiugró részek eltávolításához szükséges paraméter a célhoz
 * \param statistical_setStddevMulThres_target : statisztikailag kiugró részek eltávolításához szükséges paraméter a célhoz
 * \param setNegative_target : statisztikailag kiugró részek eltávolításához szükséges paraméter a célhoz
 * \param radius_setMinNeighborsInRadius_source : sugár alapú kiugró részek eltávolításához szükséges paraméter a forráshoz
 * \param setRadiusSearch_source : sugár alapú kiugró részek eltávolításához szükséges paraméter a forráshoz
 * \param radius_setMinNeighborsInRadius_target : sugár alapú kiugró részek eltávolításához szükséges paraméter a célhoz
 * \param setRadiusSearch_target : sugár alapú kiugró részek eltávolításához szükséges paraméter a célhoz
 * \param setSearchRadius_source : simításhoz szükséges sugár paraméter nagysága a forráshoz
 * \param setSearchRadius_target : simításhoz szükséges sugár paraméter nagysága a célhoz
 * \return
 */
int
PCLViewerXModel::Preprocessing(bool downsampling, bool downsampling_source, bool downsampling_target, bool voxeldownsampling, bool outliersremoval, bool outliersremoval_source, bool outliersremoval_target, bool removal_statistical,
                               bool smoothing, bool smoothing_source, bool smoothing_target,
     //downsampling
                              double randomsampling_setsample_source, double randomsampling_setsample_target,
                              double voxel_leaf_source, double voxel_leaf_target,
     //outliersrejection
                              int statistical_setmeanK_source, double statistical_setStddevMulThres_source, bool setNegative_source,
                              int statistical_setmeanK_target, double statistical_setStddevMulThres_target, bool setNegative_target,
     //radiusrejection
                              int radius_setMinNeighborsInRadius_source, double setRadiusSearch_source,
                              int radius_setMinNeighborsInRadius_target, double setRadiusSearch_target,
     //Smoothing
                              double setSearchRadius_source, double setSearchRadius_target

                              )
{
    if(_preprocessed_src->points.size() == 0 || _preprocessed_tgt->points.size() == 0)
        return -4;

    if(downsampling)
    {
        emit SendMessage("Preprocessing:Source Cloud had " + QString::number(_preprocessed_src->points.size()) + "before downsampling!\nPreprocessing: Target Cloud had " + QString::number(_preprocessed_tgt->points.size()) + "before downsampling!\n");

        if(voxeldownsampling)
        {
            if(downsampling_source)
            {
                if(voxel_leaf_source >= (ComputeCloudDiameter(_preprocessed_src)/7))
                {
                    emit SendMessage("Downsampling: your params are too large for this source cloud. Please give lower param!\n");
                        return -1;
                }

                if(DownSamplingBasedOnVoxelGrid(_preprocessed_src, *_preprocessed_src, voxel_leaf_source)!=0)
                    return -1;
            }
            if(downsampling_target)
            {
                if(voxel_leaf_target >= ComputeCloudDiameter(_preprocessed_tgt)/7)
                {
                    emit SendMessage("Downsampling: your params are too large for this source cloud. Please give lower param!\n");
                        return -1;

                }
                if(DownSamplingBasedOnVoxelGrid(_preprocessed_tgt, *_preprocessed_tgt, voxel_leaf_target)!=0)
                    return -1;
            }

        }
        else
        {
            if(downsampling_source)
            {
                if(randomsampling_setsample_source >= _preprocessed_src->points.size() || randomsampling_setsample_source < _preprocessed_src->points.size()*0.1)
                {
                    emit SendMessage("Downsampling: your params are too large for this source cloud. Please give lower param!\n");
                        return -1;


                }
                if(DownSamplingBasedOnRandomSampling(_preprocessed_src, *_preprocessed_src,randomsampling_setsample_source)!=0)
                    return -1;

            }
            if(downsampling_target)
            {
                if(randomsampling_setsample_target >= _preprocessed_tgt->points.size() || randomsampling_setsample_target < _preprocessed_tgt->points.size()*0.1)
                {
                    emit SendMessage("Downsampling: your params are too large for this target cloud. Please give lower param!\n");
                        return -1;

                }
                if(DownSamplingBasedOnRandomSampling(_preprocessed_tgt, *_preprocessed_tgt, randomsampling_setsample_target) != 0)
                    return -1;

            }

        }
        emit SendMessage("Preprocessing: Source Cloud has " + QString::number(_preprocessed_src->points.size()) + "after downsampling!\nPreprocessing: Target Cloud has " + QString::number(_preprocessed_tgt->points.size()) + "after downsampling!\n");
    }
    if(outliersremoval)
    {
        emit SendMessage("Preprocessing: Source Cloud had " + QString::number(_preprocessed_src->points.size()) + "before outliers rejection!\nPreprocessing: Target Cloud had " + QString::number(_preprocessed_tgt->points.size()) + "before outliers rejection!\n");

        if(removal_statistical)
        {
            if(outliersremoval_source)
            {
                if(statistical_setStddevMulThres_source >= ComputeCloudDiameter(_preprocessed_src)/4 )
                {
                    emit SendMessage("Outliers Removal: your params are too large for this source cloud. Please give lower param!\n");
                    return -2;
                }
                if(OutliersRemovalBasedOnStatistical(_preprocessed_src, *_preprocessed_src,statistical_setmeanK_source, statistical_setStddevMulThres_source,setNegative_source)!=0)
                    return -2;
            }
            if(outliersremoval_target)
            {
                if(statistical_setStddevMulThres_target >= ComputeCloudDiameter(_preprocessed_tgt)/4)
                {
                    emit SendMessage("Outliers Removal: your params are too large for this source cloud. Please give lower param!\n");
                    return -2;

                }
                if(OutliersRemovalBasedOnStatistical(_preprocessed_tgt, *_preprocessed_tgt,statistical_setmeanK_target, statistical_setStddevMulThres_target,setNegative_target) != 0)
                    return -2;
            }
        }
        else
        {
            if(outliersremoval_source)
            {
                if(setRadiusSearch_source >= ComputeCloudDiameter(_preprocessed_src)/4 )
                {
                    emit SendMessage("Outliers Removal: your params are too large for this source cloud. Please give lower param!\n");
                    return -2;
                }

                if(OutliersRemovalBasedOnRadius(_preprocessed_src,*_preprocessed_src,radius_setMinNeighborsInRadius_source,setRadiusSearch_source) != 0)
                    return -2;
            }
            if(outliersremoval_target)
            {
                if(setRadiusSearch_target >= ComputeCloudDiameter(_preprocessed_tgt)/4
                        )
                {
                    emit SendMessage("Outliers Removal: your params are too large for this source cloud. Please give lower param!\n");
                    return -2;

                }
                if(OutliersRemovalBasedOnRadius(_preprocessed_tgt,*_preprocessed_tgt, radius_setMinNeighborsInRadius_target,setRadiusSearch_target) != 0)
                    return -2;
            }
        }
        emit SendMessage("Preprocessing: Source Cloud had " + QString::number(_preprocessed_src->points.size()) + "after outliers rejection!\nPreprocessing: Target Cloud had " + QString::number(_preprocessed_tgt->points.size()) + "after outliers rejection!\n");
    }
    if(smoothing)
    {
        emit SendMessage("Preprocessing: Source Cloud had " + QString::number(_preprocessed_src->points.size()) + "before smoothing!\nPreprocessing: Target Cloud had " + QString::number(_preprocessed_tgt->points.size()) + "before smoothing!\n");

        if(smoothing_source)
        {
            if(setSearchRadius_source >= ComputeCloudDiameter(_preprocessed_src)/7)
            {
                emit SendMessage("Smoothing: your params are too large for this source cloud. Please give lower param!\n");
                return -3;

            }

            if(SmoothingBasedOnMovingLeastSquares(_preprocessed_src,*_preprocessed_src,setSearchRadius_source) != 0)
                return -3;
        }
        if(smoothing_target)
        {
            if(setSearchRadius_target >= ComputeCloudDiameter(_preprocessed_tgt)/7)
            {
                emit SendMessage("Smoothing: your params are too large for this source cloud. Please give lower param!\n");
                return -3;
            }
            if(SmoothingBasedOnMovingLeastSquares(_preprocessed_tgt, *_preprocessed_tgt,setSearchRadius_target) != 0)
                return -3;

        }
        emit SendMessage("Preprocessing: Source Cloud had " + QString::number(_preprocessed_src->points.size()) + "after smoothing!\nPreprocessing: Target Cloud had " + QString::number(_preprocessed_tgt->points.size()) + "after smoothing!\n");
    }
    return 0;
}
