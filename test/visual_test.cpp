
#include "osg/Node"
#include "osg/Geometry"
#include "osgDB/ReadFile"
#include "osgViewer/Viewer"

int main(int argc, char* argv[])
{
    if(argc < 2)
        return 1;

    osg::ref_ptr<osg::Node> file_node = osgDB::readNodeFile(argv[1]);
    if(!file_node)
        return 2;

    if(file_node->asGroup())
    {
        std::cout << "Got a Group" << std::endl;

        for(size_t j=0, last_j=file_node->asGroup()->getNumChildren(); j<last_j; ++j)
        {
            osg::ref_ptr<osg::Geode> geode = file_node->asGroup()->getChild(j)->asGeode();
            if(geode)
            {
                std::cout << "Got a Geode" << std::endl;
                for(size_t i=0, last_i=geode->getNumDrawables(); i<last_i; ++i)
                {
                    osg::ref_ptr<osg::Geometry> G = geode->getDrawable(i)->asGeometry();

                    if(G)
                    {
                        osg::ref_ptr<osg::Vec3Array> V = dynamic_cast<osg::Vec3Array*>
                                (G->getVertexArray());
                        if(V)
                        {
                            std::cout << "Got vertex array (" << V->size() << ")" << std::endl;
                        }

                        osg::ref_ptr<osg::Vec3Array> N = dynamic_cast<osg::Vec3Array*>
                                (G->getNormalArray());
                        if(N)
                        {
                            std::cout << "Got normal array (" << N->size() << ")" << std::endl;
                            for(size_t k=0; k<N->size(); ++k)
                            {
                                double x=(*N)[k][0], y=(*N)[k][1], z=(*N)[k][2];
                                std::cout << "#" << k << "\t(" << sqrt(x*x+y*y+z*z) << ")\t"
                                          << x << " " << y << " " << z << "\n";
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        std::cerr << "Error: Couldn't convert to group!" << std::endl;
    }

    osgViewer::Viewer viewer;
    viewer.setSceneData(file_node);
    viewer.run();
}
