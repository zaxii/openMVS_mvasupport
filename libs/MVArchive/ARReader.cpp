//
// Created by song.1634 on 1/25/2022.
//

#include "ARReader.h"

namespace ARCH {
    bool LoadInterface(MVS::Scene &scene, const std::string fileName) {
        MVSA::Interface obj;
        // serialize in
        if (!MVSA::MVArchive::SerializeLoad(obj, fileName)) return false;

        scene.Release();
        ////////////////////
        /// Import platforms
        scene.platforms.Resize(obj.platforms.size());
        FOREACH(i, scene.platforms) {
            const MVSA::Interface::Platform plat = obj.platforms[i];
            MVS::Platform &platform = scene.platforms[i];
            platform.name = plat.name;

            platform.cameras.Resize(plat.cameras.size());
            FOREACH(j, platform.cameras) {
                const MVSA::Interface::Platform::Camera &cam = plat.cameras[j];
                MVS::Platform::Camera &camera = platform.cameras[j];
                camera.K = cam.K;
                camera.R = cam.R;
                camera.C = cam.C;
            }
            platform.poses.Resize(plat.poses.size());
            FOREACH(j, platform.poses) {
                const MVSA::Interface::Platform::Pose p = plat.poses[j];
                MVS::Platform::Pose &pose = platform.poses[j];
                pose.R = p.R;
                pose.C = p.C;
            }
        }
        /// Import platforms
        ////////////////////

        /////////////////
        /// Import images
        scene.images.Resize(obj.images.size());
        FOREACH(i, scene.images) {
            const MVSA::Interface::Image &image = obj.images[i];
            MVS::Image &imageData = scene.images[i];
            imageData.name = MAKE_PATH_FULL(WORKING_FOLDER_FULL, image.name);

            imageData.poseID = image.poseID;
            imageData.platformID = image.platformID;
            imageData.cameraID = image.cameraID;
            imageData.width = image.width;
            imageData.height = image.height;

            imageData.UpdateCamera(scene.platforms);
        }
        /// Import images
        /////////////////

        ///////////////////////
        /// Import point cloud
        scene.pointcloud.points.Resize(obj.vertices.size());
        scene.pointcloud.pointViews.Resize(obj.vertices.size());
        scene.pointcloud.pointWeights.Resize(obj.vertices.size());

        FOREACH(i, scene.pointcloud.points) {
            const MVSA::Interface::Vertex &vertex = obj.vertices[i];
            MVS::PointCloud::Point &point = scene.pointcloud.points[i];
            MVS::PointCloud::ViewArr &views = scene.pointcloud.pointViews[i];
            MVS::PointCloud::WeightArr &weights = scene.pointcloud.pointWeights[i];
            ASSERT(sizeof(vertex.X.x) == sizeof(point.x));
            point = vertex.X;
            views.Resize(vertex.views.size());
            weights.Resize(vertex.views.size());

            FOREACH(j, views) {
                views[j] = vertex.views[j].imageID;
                weights[j] = vertex.views[j].confidence;
            }
        }

        scene.pointcloud.normals.Resize(obj.verticesNormal.size());
        FOREACH(i, scene.pointcloud.normals) {
            const MVSA::Interface::Normal &vertexNormal = obj.verticesNormal[i];
            MVS::PointCloud::Normal &normal = scene.pointcloud.normals[i];
            normal = vertexNormal.n;
        }

        scene.pointcloud.colors.Resize(obj.verticesColor.size());
        FOREACH(i, scene.pointcloud.colors) {
            const MVSA::Interface::Color &vertexColor = obj.verticesColor[i];
            MVS::PointCloud::Color &color = scene.pointcloud.colors[i];
            color.r = vertexColor.c.x;
            color.g = vertexColor.c.y;
            color.b = vertexColor.c.z;
        }
        /// Import point cloud
        //////////////////////

        //////////////////////
        /// Import Mesh
        int nMeshVertices = obj.mesh.vertices.size();
        int nMeshFaces = obj.mesh.faces.size();
        int nMeshTextures = obj.mesh.textureDiffuses.size();
        int nMeshTexCoords = obj.mesh.faceTexcoords.size();

        scene.mesh.vertices.Resize(nMeshVertices);
        scene.mesh.faces.Resize(nMeshFaces);

        for (int i = 0; i < nMeshVertices; ++i) {
            const MVSA::Interface::Mesh::Vertex &vertex = obj.mesh.vertices[i];
            MVS::Mesh::Vertex &point = scene.mesh.vertices[i];
            point = vertex.X;
        }

        for (int i = 0; i < nMeshFaces; ++i) {
            const MVSA::Interface::Mesh::Face &face = obj.mesh.faces[i];
            MVS::Mesh::Face &f = scene.mesh.faces[i];
            f = face.f;
        }

        if (!obj.mesh.vertexNormals.empty()) {
            scene.mesh.vertexNormals.Resize(nMeshVertices);
            for (int i = 0; i < nMeshVertices; ++i) {
                const MVSA::Interface::Mesh::Normal &vertexNormal =
                        obj.mesh.vertexNormals[i];
                MVS::Mesh::Normal &normal = scene.mesh.vertexNormals[i];
                normal = vertexNormal.n;
            }
        }

        if (!obj.mesh.vertexVertices.empty()) {
            scene.mesh.vertexVertices.Resize(nMeshVertices);
            for (int i = 0; i < nMeshVertices; ++i) {
                const MVSA::Interface::Mesh::VertexIdxArr &vertexVertex =
                        obj.mesh.vertexVertices[i];
                MVS::Mesh::VertexIdxArr &vv = scene.mesh.vertexVertices[i];
                vv.Resize(vertexVertex.size());
                for (int j = 0, jend = vv.GetSize(); j < jend; ++j) {
                    vv[j] = vertexVertex[j];
                }
            }
        }

        if (!obj.mesh.vertexFaces.empty()) {
            scene.mesh.vertexFaces.Resize(nMeshVertices);
            for (int i = 0; i < nMeshVertices; ++i) {
                const MVSA::Interface::Mesh::FaceIdxArr &vertexFaces =
                        obj.mesh.vertexFaces[i];
                MVS::Mesh::FaceIdxArr &vf = scene.mesh.vertexFaces[i];
                vf.Resize(vertexFaces.size());
                for (int j = 0, jend = vf.GetSize(); j < jend; ++j) {
                    vf[j] = vertexFaces[j];
                }
            }
        }

        if (!obj.mesh.vertexBoundary.empty()) {
            scene.mesh.vertexBoundary.Resize(nMeshVertices);
            for (int i = 0; i < nMeshVertices; ++i) {
                scene.mesh.vertexBoundary[i] = obj.mesh.vertexBoundary[i] > 0;
            }
        }

        if (!obj.mesh.faceNormals.empty()) {
            scene.mesh.faceNormals.Resize(nMeshFaces);
            for (int i = 0; i < nMeshFaces; ++i) {
                const MVSA::Interface::Mesh::Normal &fNormal = obj.mesh.faceNormals[i];
                MVS::Mesh::Normal &normal = scene.mesh.faceNormals[i];
                normal = fNormal.n;
            }
        }

        if (!obj.mesh.faceTexcoords.empty()) {
            scene.mesh.faceTexcoords.Resize(nMeshTexCoords);
            for (int i = 0; i < nMeshTexCoords; ++i) {
                const MVSA::Interface::Mesh::TexCoord &texCoord =
                        obj.mesh.faceTexcoords[i];
                MVS::Mesh::TexCoord &tc = scene.mesh.faceTexcoords[i];
                tc = texCoord.tc;
            }
        }

#ifdef OFFICIAL_OPENMVS
        if(!obj.mesh.textureDiffuses.empty()) {
            const MVSA::Interface::Mesh::Texture &texture =
                    obj.mesh.textureDiffuses[0];
            SEACAVE::Image8U3 &image = scene.mesh.textureDiffuse;
            image.create(texture.height, texture.width);
            size_t datalength = texture.width * texture.height * 3;
            ASSERT(datalength == (image.row_stride() * image.height()));
            std::copy(texture.data.data(), texture.data.data() + datalength,
                      image.data);
        }
#else // OFFICIAL_OPENMVS
        if (!obj.mesh.faceMapIdxs.empty()) {
    scene.mesh.faceMapIdxs.Resize(nMeshFaces);
    for (int i = 0; i < nMeshFaces; ++i) {
      scene.mesh.faceMapIdxs[i] = obj.mesh.faceMapIdxs[i];
    }
  }

  if (!obj.mesh.textureDiffuses.empty()) {
    scene.mesh.textureDiffuses.Resize(nMeshTextures);
    for (int i = 0; i < nMeshTextures; ++i) {
      const MVSA::Interface::Mesh::Texture &texture =
          obj.mesh.textureDiffuses[i];
      SEACAVE::Image8U3 &image = scene.mesh.textureDiffuses[i];
      image.create(texture.height, texture.width);
      size_t datalength = texture.width * texture.height * 3;
      ASSERT(datalength == (image.row_stride() * image.height()));
      std::copy(texture.data.data(), texture.data.data() + datalength,
                image.data);
    }
  }
#endif //OFFICIAL_OPENMVS
        /// Import Mesh
        //////////////////////
        return true;
    }

    bool AutoLoadScene(MVS::Scene &scene, const std::string fileName,
                       bool bImport) {
        const String ext(Util::getFileExt(fileName).ToLower());
        if (ext == _T(".mvs")) {
            if (!scene.Load(fileName, bImport)) return false;
        } else if (ext == _T(".mva")) {
            if (!LoadInterface(scene, fileName)) return false;
        }
        return true;
    }
}  // namespace ARCH