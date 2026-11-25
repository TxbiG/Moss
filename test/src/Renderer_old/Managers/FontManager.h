

class [[nodiscard]] Font {
public:
    virtual ~Font() = default;

    virtual void loadFromFile(const std::string& filepath, int size) = 0;
    virtual void drawText(const std::string& text, Float2 position, float scale, const Color& color) = 0;
    virtual float measureTextWidth(const std::string& text, float scale) const = 0;
    virtual float getLineHeight(float scale) const = 0;
};



class FontManager {
public:
    void loadFont(const std::string& id, const std::string& filePath, int size);
    Font* getFont(const std::string& id);

private:
    std::unordered_map<std::string, std::unique_ptr<Font>> fonts;
};