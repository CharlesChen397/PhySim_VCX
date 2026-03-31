#include "Labs/1-RigidBody/PrimitiveMesh.h"

#include <cmath>

namespace VCX::Labs::RigidBody {

    namespace {
        MeshData buildBox(RigidBody const & body) {
            MeshData mesh;
            mesh.Vertices.resize(8);

            Eigen::Vector3f const hx = body.Q * Eigen::Vector3f(0.5f * body.Dim.x(), 0.f, 0.f);
            Eigen::Vector3f const hy = body.Q * Eigen::Vector3f(0.f, 0.5f * body.Dim.y(), 0.f);
            Eigen::Vector3f const hz = body.Q * Eigen::Vector3f(0.f, 0.f, 0.5f * body.Dim.z());

            mesh.Vertices[0] = body.X - hx + hy + hz;
            mesh.Vertices[1] = body.X + hx + hy + hz;
            mesh.Vertices[2] = body.X + hx + hy - hz;
            mesh.Vertices[3] = body.X - hx + hy - hz;
            mesh.Vertices[4] = body.X - hx - hy + hz;
            mesh.Vertices[5] = body.X + hx - hy + hz;
            mesh.Vertices[6] = body.X + hx - hy - hz;
            mesh.Vertices[7] = body.X - hx - hy - hz;

            mesh.Triangles = {
                0,
                1,
                2,
                0,
                2,
                3,
                1,
                0,
                4,
                1,
                4,
                5,
                1,
                5,
                6,
                1,
                6,
                2,
                2,
                6,
                7,
                2,
                7,
                3,
                0,
                3,
                7,
                0,
                7,
                4,
                4,
                6,
                5,
                4,
                7,
                6,
            };

            mesh.Lines = {
                0,
                1,
                1,
                2,
                2,
                3,
                3,
                0,
                4,
                5,
                5,
                6,
                6,
                7,
                7,
                4,
                0,
                4,
                1,
                5,
                2,
                6,
                3,
                7,
            };
            return mesh;
        }

        MeshData buildSphere(RigidBody const & body, int slices = 16, int stacks = 10) {
            MeshData mesh;
            for (int i = 0; i <= stacks; ++i) {
                float const v   = float(i) / float(stacks);
                float const phi = v * float(M_PI);
                for (int j = 0; j <= slices; ++j) {
                    float const     u     = float(j) / float(slices);
                    float const     theta = u * 2.f * float(M_PI);
                    Eigen::Vector3f local(
                        body.Radius * std::sin(phi) * std::cos(theta),
                        body.Radius * std::cos(phi),
                        body.Radius * std::sin(phi) * std::sin(theta));
                    mesh.Vertices.push_back(body.X + body.Q * local);
                }
            }

            auto idx = [slices](int i, int j) {
                return i * (slices + 1) + j;
            };

            for (int i = 0; i < stacks; ++i) {
                for (int j = 0; j < slices; ++j) {
                    std::uint32_t a = idx(i, j);
                    std::uint32_t b = idx(i + 1, j);
                    std::uint32_t c = idx(i + 1, j + 1);
                    std::uint32_t d = idx(i, j + 1);

                    mesh.Triangles.insert(mesh.Triangles.end(), { a, b, c, a, c, d });
                    mesh.Lines.insert(mesh.Lines.end(), { a, b, a, d });
                }
            }
            return mesh;
        }

        MeshData buildCylinder(RigidBody const & body, int segments = 18) {
            MeshData    mesh;
            float const h = 0.5f * body.Height;

            mesh.Vertices.reserve(2 * segments + 2);
            for (int i = 0; i < segments; ++i) {
                float const           t = (2.f * float(M_PI) * i) / float(segments);
                Eigen::Vector3f const ring(body.Radius * std::cos(t), 0.f, body.Radius * std::sin(t));
                mesh.Vertices.push_back(body.X + body.Q * (ring + Eigen::Vector3f(0.f, h, 0.f)));
                mesh.Vertices.push_back(body.X + body.Q * (ring - Eigen::Vector3f(0.f, h, 0.f)));
            }

            std::uint32_t topCenter = static_cast<std::uint32_t>(mesh.Vertices.size());
            mesh.Vertices.push_back(body.X + body.Q * Eigen::Vector3f(0.f, h, 0.f));
            std::uint32_t bottomCenter = static_cast<std::uint32_t>(mesh.Vertices.size());
            mesh.Vertices.push_back(body.X + body.Q * Eigen::Vector3f(0.f, -h, 0.f));

            for (int i = 0; i < segments; ++i) {
                int           ni = (i + 1) % segments;
                std::uint32_t t0 = 2 * i;
                std::uint32_t b0 = 2 * i + 1;
                std::uint32_t t1 = 2 * ni;
                std::uint32_t b1 = 2 * ni + 1;

                mesh.Triangles.insert(mesh.Triangles.end(), { t0, b0, b1, t0, b1, t1 });
                mesh.Triangles.insert(mesh.Triangles.end(), { topCenter, t1, t0 });
                mesh.Triangles.insert(mesh.Triangles.end(), { bottomCenter, b0, b1 });

                mesh.Lines.insert(mesh.Lines.end(), { t0, t1, b0, b1, t0, b0 });
            }

            return mesh;
        }
    } // namespace

    MeshData BuildBodyMesh(RigidBody const & body) {
        switch (body.Shape) {
        case ShapeType::Box:
            return buildBox(body);
        case ShapeType::Sphere:
            return buildSphere(body);
        case ShapeType::Cylinder:
            return buildCylinder(body);
        }
        return buildBox(body);
    }

} // namespace VCX::Labs::RigidBody
