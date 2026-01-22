


class [[nodiscard]] MOSS_API Model {
public:
    Model();
    ~Model();

    update();

    draw(Mat44 matrixprojection);

private:
    std::vector<Mesh> m_meshes;
}