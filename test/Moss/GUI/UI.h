


UI


PipelineState uiPipeline(renderer, &uiVertexShader, uiInputDesc, uiInputDescCount, &uiPixelShader, 
    EDrawPass::Normal, EFillMode::Solid, ETopology::Triangle, EDepthTest::Off, EBlendMode::Alpha, ECullMode::None);




class Canvas {
public:
    ~UIRenderer() {}

    // Basic draw calls
    void drawRect(const Rect& rect, const Color& color) = 0;
    void drawText(const Vec2& pos, const std::string& text, const Color& color) = 0;
    void drawTexture(const Rect& rect, unsigned textureID) = 0;

    // Scissor for clipping
    void pushScissor(const Rect& rect) = 0;
    void popScissor() = 0;
};

class UIWidget {
public:
    Vec2 position {0,0};
    Vec2 size {100,30};
    bool visible = true;
    bool enabled = true;
    UIWidget* parent = nullptr;

    ~UIWidget() {}
    void draw(UIRenderer& renderer) = 0;
    void update(float dt) {}
    bool onMouseClick(const Vec2& mousePos) { return false; }

    bool pointInRect(const Vec2& p) const {
        return p.x >= position.x && p.x <= position.x + size.x && p.y >= position.y && p.y <= position.y + size.y;
    }
};


class Text : UIWidget { }
class RichText : UIWidget { }

class Button : public UIWidget {
public:
    std::string text = "Button";
    Color bgColor {0.2f,0.2f,0.2f,1};
    std::function<void()> onClick;

    void draw(UIRenderer& r) override {
        r.drawRect({(int)position.x, (int)position.y, (int)size.x, (int)size.y}, bgColor);
        r.drawText({position.x+5, position.y+5}, text, {1,1,1,1});
    }

    bool onMouseClick(const Vec2& mousePos) override {
        if (pointInRect(mousePos)) {
            if (onClick) onClick();
            return true;
        }
        return false;
    }
};

class ButtonImage : public UIWidget { };
class Toggle : public UIWidget { };
class Checkbox : public UIWidget { };
class RadioButton : public UIWidget { };


class TextureRect : public UIWidget {
public:
    unsigned textureID = 0;
    void draw(UIRenderer& r) override {
        r.drawTexture({(int)position.x, (int)position.y, (int)size.x, (int)size.y}, textureID);
    }
};

class ColorRect : public UIWidget {
public:
    Color color {1,1,1,1};
    void draw(UIRenderer& r) override {
        r.drawRect({(int)position.x, (int)position.y, (int)size.x, (int)size.y}, color);
    }
};



class ProgressBar : public UIWidget { };

class Slider : public UIWidget { };


class UIContainer : public UIWidget {
public:
    std::vector<std::unique_ptr<UIWidget>> children;

    void addChild(std::unique_ptr<UIWidget> child) {
        child->parent = this;
        children.push_back(std::move(child));
        layout();
    }

    void draw(UIRenderer& renderer) override {
        for (auto& child : children)
            if (child->visible)
                child->draw(renderer);
    }

    void update(float dt) override {
        for (auto& child : children)
            child->update(dt);
    }

protected:
    void layout() {}
};

class CenterContainer : public UIContainer {
protected:
    void layout() override {
        if (children.empty()) return;
        auto& c = children[0];
        c->position = { position.x + (size.x - c->size.x) / 2,
                        position.y + (size.y - c->size.y) / 2 };
    }
};

class VContainer : public UIContainer {
public:
    float spacing = 5;
protected:
    void layout() override {
        float y = position.y;
        for (auto& c : children) {
            c->position = { position.x, y };
            y += c->size.y + spacing;
        }
    }
};

class HContainer : public UIContainer {
public:
    float spacing = 5;
protected:
    void layout() override {
        float x = position.x;
        for (auto& c : children) {
            c->position = { x, position.y };
            x += c->size.x + spacing;
        }
    }
};

class GridContainer : public UIContainer { };

class VScrollContainer : public UIContainer { };
class HScrollContainer : public UIContainer { };
class MarginContainer : public UIContainer { };
class CenterContainer : public UIContainer { };