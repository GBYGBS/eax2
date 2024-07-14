#include "includes.h"

namespace render {
	Font menu;;
	Font menu_shade;;
	Font esp;;
	Font esp2;;
	Font esp_small;;
	Font esp_small2;;
	Font esp_name;;
	Font esp_other;;
	Font hud;;
	Font cs;;
	Font indicator;;
	Font indicator2;;
	Font console;;
	Font manualaa;;
	Font output;;
}

void render::init( ) {
	menu = Font(XOR("Tahoma"), 12, FW_NORMAL, FONTFLAG_DROPSHADOW);
	menu_shade = Font( XOR( "Tahoma" ), 12, FW_NORMAL, FONTFLAG_DROPSHADOW );
	esp        = Font( XOR( "Tahoma" ), 12, FW_NORMAL, FONTFLAG_ANTIALIAS | FONTFLAG_DROPSHADOW );
	esp2       = Font( XOR( "Verdana" ), 12, FW_NORMAL, FONTFLAG_ANTIALIAS | FONTFLAG_DROPSHADOW);
	esp_small  = Font( XOR( "Smallest Pixel-7" ), 10, FW_NORMAL, FONTFLAG_OUTLINE);
	esp_small2 = Font( XOR( "Small Fonts" ), 9, FW_NORMAL, FONTFLAG_OUTLINE);
	esp_name   = Font( XOR( "Verdana" ), 12, FW_NORMAL, FONTFLAG_DROPSHADOW);
	esp_other  = Font( XOR( "Verdana" ), 12, FW_BOLD, FONTFLAG_DROPSHADOW);
	hud        = Font( XOR( "Tahoma" ), 16, FW_NORMAL, FONTFLAG_ANTIALIAS );
	cs         = Font( XOR( "Undefeated" ), 14, FW_NORMAL, FONTFLAG_ANTIALIAS | FONTFLAG_DROPSHADOW );
	indicator  = Font( XOR( "Verdana" ), 26, FW_BOLD, FONTFLAG_ANTIALIAS | FONTFLAG_DROPSHADOW );
	indicator2 = Font( XOR( "Bahnschrift" ), 28, FW_MEDIUM, FONTFLAG_ANTIALIAS | FONTFLAG_DROPSHADOW);
	console    = Font( XOR( "Lucida Console" ), 10, FW_DONTCARE, FONTFLAG_DROPSHADOW);
	manualaa   = Font( XOR( "Acta Symbols W95 Arrows" ), 16, FW_NORMAL, FONTFLAG_ANTIALIAS);
	output	   = Font( XOR( "Verdana"), 12, FW_BOLD, FONTFLAG_OUTLINE);
}

bool render::WorldToScreen( const vec3_t& world, vec2_t& screen ) {
	float w;

	const VMatrix& matrix = g_csgo.m_engine->WorldToScreenMatrix( );

	// check if it's in view first.
    // note - dex; w is below 0 when world position is around -90 / +90 from the player's camera on the y axis.
	w = matrix[ 3 ][ 0 ] * world.x + matrix[ 3 ][ 1 ] * world.y + matrix[ 3 ][ 2 ] * world.z + matrix[ 3 ][ 3 ];
	if( w < 0.001f )
		return false;

	// calculate x and y.
	screen.x = matrix[ 0 ][ 0 ] * world.x + matrix[ 0 ][ 1 ] * world.y + matrix[ 0 ][ 2 ] * world.z + matrix[ 0 ][ 3 ];
	screen.y = matrix[ 1 ][ 0 ] * world.x + matrix[ 1 ][ 1 ] * world.y + matrix[ 1 ][ 2 ] * world.z + matrix[ 1 ][ 3 ];

	screen /= w;

	// calculate screen position.
	screen.x = ( g_cl.m_width / 2 ) + ( screen.x * g_cl.m_width ) / 2;
	screen.y = ( g_cl.m_height / 2 ) - ( screen.y * g_cl.m_height ) / 2;

	return true;
}



void render::RoundedBoxStatic(int x, int y, int w, int h, int radius, Color col)
{
	Vertex round[64];

	for (int i = 0; i < 4; i++)
	{
		int _x = x + ((i < 2) ? (w - radius) : radius);
		int _y = y + ((i % 3) ? (h - radius) : radius);

		float a = 90.f * i;

		for (int j = 0; j < 16; j++)
		{
			float _a = DEG2RAD(a + j * 6.f);

			round[(i * 16) + j] = Vertex(vec2_t(_x + radius * sin(_a), _y - radius * cos(_a)));
		}
	}
	g_csgo.m_surface->DrawSetColor(col);
	g_csgo.m_surface->DrawTexturedPolygon(64, round);
}

void render::RoundedBoxStaticOutline(int x, int y, int w, int h, int radius, Color col)
{
	Vertex round[64];

	for (int i = 0; i < 4; i++)
	{
		int _x = x + ((i < 2) ? (w - radius) : radius);
		int _y = y + ((i % 3) ? (h - radius) : radius);

		float a = 90.f * i;

		for (int j = 0; j < 16; j++)
		{
			float _a = DEG2RAD(a + j * 6.f);

			round[(i * 16) + j] = Vertex(vec2_t(_x + radius * sin(_a), _y - radius * cos(_a)));
		}
	}
	g_csgo.m_surface->DrawSetColor(col);
	g_csgo.m_surface->DrawTexturedPolyLine(64, round);
}

void render::WorldCircleOutline(vec3_t origin, float radius, float angle, Color color) {
	std::vector< Vertex > vertices{};

	float step = (1.f / radius) + math::deg_to_rad(angle);

	float lat = 1.f;
	vertices.clear();

	for (float lon{}; lon < math::pi_2; lon += step) {
		vec3_t point{
			origin.x + (radius * std::sin(lat) * std::cos(lon)),
			origin.y + (radius * std::sin(lat) * std::sin(lon)),
			origin.z + (radius * std::cos(lat) - (radius / 2))
		};

		vec2_t screen;
		if (WorldToScreen(point, screen))
			vertices.emplace_back(screen);
	}
	static int texture = g_csgo.m_surface->CreateNewTextureID(false);

	g_csgo.m_surface->DrawSetTextureRGBA(texture, &colors::white, 1, 1);
	g_csgo.m_surface->DrawSetColor(color);

	//g_csgo.m_surface->DrawSetTexture(texture);
	//g_csgo.m_surface->DrawTexturedPolygon(vertices.size(), vertices.data());

	g_csgo.m_surface->DrawTexturedPolyLine(vertices.size(), vertices.data());
}

void render::round_rect(int x, int y, int w, int h, int r, Color color) {
	Vertex round[64];

	for (int i = 0; i < 4; i++) {
		int _x = x + ((i < 2) ? (w - r) : r);
		int _y = y + ((i % 3) ? (h - r) : r);

		float a = 90.f * i;

		for (int j = 0; j < 16; j++) {
			float _a = math::deg_to_rad(a + j * 6.f);

			round[(i * 16) + j] = Vertex(vec2_t(_x + r * sin(_a), _y - r * cos(_a)));
		}
	}

	g_csgo.m_surface->DrawSetColor(color);
	g_csgo.m_surface->DrawTexturedPolygon(64, round);
}

void render::line( vec2_t v0, vec2_t v1, Color color ) {
	g_csgo.m_surface->DrawSetColor( color );
	g_csgo.m_surface->DrawLine( v0.x, v0.y, v1.x, v1.y );
}

void render::line( int x0, int y0, int x1, int y1, Color color ) {
	g_csgo.m_surface->DrawSetColor( color );
	g_csgo.m_surface->DrawLine( x0, y0, x1, y1 );
}

void render::rect( int x, int y, int w, int h, Color color ) {
	g_csgo.m_surface->DrawSetColor( color );
	g_csgo.m_surface->DrawOutlinedRect( x, y, x + w, y + h );
}

void render::rect_filled( int x, int y, int w, int h, Color color ) {
	g_csgo.m_surface->DrawSetColor( color );
	g_csgo.m_surface->DrawFilledRect( x, y, x + w, y + h );
}

void render::RectFilled(vec2_t pos, vec2_t size, Color color) {
	render::rect_filled(pos.x, pos.y, size.x, size.y, color);
}

void render::rect_filled_fade( int x, int y, int w, int h, Color color, int a1, int a2 ) {
	g_csgo.m_surface->DrawSetColor( color );
	g_csgo.m_surface->DrawFilledRectFade( x, y, x + w, y + h, a1, a2, false );
}

void render::rect_outlined( int x, int y, int w, int h, Color color, Color color2 ) {
	rect( x, y, w, h, color );
	rect( x - 1, y - 1, w + 2, h + 2, color2 );
	rect( x + 1, y + 1, w - 2, h - 2, color2 );
}

void render::circle( int x, int y, int radius, int segments, Color color ) {
	static int texture = g_csgo.m_surface->CreateNewTextureID( true );

	g_csgo.m_surface->DrawSetTextureRGBA( texture, &colors::white, 1, 1 );
	g_csgo.m_surface->DrawSetColor( color );
	g_csgo.m_surface->DrawSetTexture( texture );

	std::vector< Vertex > vertices{};

	float step = math::pi_2 / segments;
	for( float i{ 0.f }; i < math::pi_2; i += step )
		vertices.emplace_back( vec2_t{ x + ( radius * std::cos( i ) ), y + ( radius * std::sin( i ) ) } );

	g_csgo.m_surface->DrawTexturedPolygon( vertices.size( ), vertices.data( ) );
}

void render::gradient( int x, int y, int w, int h, Color color1, Color color2 ) {
	g_csgo.m_surface->DrawSetColor( color1 );
	g_csgo.m_surface->DrawFilledRectFade( x, y, x + w, y + h, color1.a( ), 0, false );

	g_csgo.m_surface->DrawSetColor( color2 );
	g_csgo.m_surface->DrawFilledRectFade( x, y, x + w, y + h, 0, color2.a( ), false );
}

void render::gradient2(int x, int y, int w, int h, Color color1, Color color2) {
	g_csgo.m_surface->DrawSetColor(color1);
	g_csgo.m_surface->DrawFilledRectFade(x, y, x + w, y + h, color1.a(), 0, false);

	g_csgo.m_surface->DrawSetColor(color2);
	g_csgo.m_surface->DrawFilledRectFade(x, y, x + w, y + h, 0, color2.a(), false);
}

void render::sphere( vec3_t origin, float radius, float angle, float scale, Color color ) {
	std::vector< Vertex > vertices{};

	// compute angle step for input radius and precision.
	float step = ( 1.f / radius ) + math::deg_to_rad( angle );

	for( float lat{}; lat < ( math::pi * scale ); lat += step ) {
		// reset.
		vertices.clear( );

		for( float lon{}; lon < math::pi_2; lon += step ) {
			vec3_t point{
				origin.x + ( radius * std::sin( lat ) * std::cos( lon ) ),
				origin.y + ( radius * std::sin( lat ) * std::sin( lon ) ),
				origin.z + ( radius * std::cos( lat ) )
			};

			vec2_t screen;
			if( WorldToScreen( point, screen ) )
				vertices.emplace_back( screen );
		}

		if( vertices.empty( ) )
			continue;

		g_csgo.m_surface->DrawSetColor( color );
		g_csgo.m_surface->DrawTexturedPolyLine( vertices.size( ), vertices.data( ) );
	}
}

void render::triangle(vec2_t point_one, vec2_t point_two, vec2_t point_three, Color color)
{
	if (!g_csgo.m_surface)
		return;

	Vertex verts[3] = {
		Vertex(point_one),
		Vertex(point_two),
		Vertex(point_three)
	};

	auto surface = g_csgo.m_surface;

	static int texture = surface->CreateNewTextureID(true);
	unsigned char buffer[4] = { 255, 255, 255, 255 };

	surface->DrawSetTextureRGBA(texture, buffer, 1, 1);
	surface->DrawSetColor(color);
	surface->DrawSetTexture(texture);

	surface->DrawTexturedPolygon(3, verts);
}

void render::onetap(vec2_t point_one, vec2_t point_two, vec2_t point_three, Color color, Color outline)
{
	Vertex verts[3] = {
		Vertex(point_one),
		Vertex(point_two),
		Vertex(point_three)
	};

	static int texture = g_csgo.m_surface->CreateNewTextureID(true);
	unsigned char buffer[4] = { 255, 255, 255, 255 };

	g_csgo.m_surface->DrawSetTextureRGBA(texture, buffer, 1, 1);

	g_csgo.m_surface->DrawSetColor(color);
	g_csgo.m_surface->DrawSetTexture(texture);

	g_csgo.m_surface->DrawTexturedPolygon(3, verts);

	g_csgo.m_surface->DrawSetColor(outline); // filled
	g_csgo.m_surface->DrawTexturedPolyLine(3, verts); // outline
}

bool render::WorldToScreen2(const vec3_t& world, vec3_t& screen) {
	float w;

	const VMatrix& matrix = g_csgo.m_engine->WorldToScreenMatrix();

	// check if it's in view first.
	// note - dex; w is below 0 when world position is around -90 / +90 from the player's camera on the y axis.
	w = matrix[3][0] * world.x + matrix[3][1] * world.y + matrix[3][2] * world.z + matrix[3][3];
	if (w < 0.001f)
		return false;

	// calculate x and y.
	screen.x = matrix[0][0] * world.x + matrix[0][1] * world.y + matrix[0][2] * world.z + matrix[0][3];
	screen.y = matrix[1][0] * world.x + matrix[1][1] * world.y + matrix[1][2] * world.z + matrix[1][3];

	screen /= w;

	// calculate screen position.
	screen.x = (g_cl.m_width / 2) + (screen.x * g_cl.m_width) / 2;
	screen.y = (g_cl.m_height / 2) - (screen.y * g_cl.m_height) / 2;

	return true;
}
#define ZERO vec3_t(0.0f, 0.0f, 0.0f)
void render::Draw3DFilledCircle(const vec3_t& origin, float radius, Color color)
{
	static auto prevScreenPos = ZERO; //-V656
	static auto step = M_PI * 2.0f / 72.0f;

	auto screenPos = ZERO;
	auto screen = ZERO;

	if (!render::WorldToScreen2(origin, screen))
		return;

	for (auto rotation = 0.0f; rotation <= M_PI * 2.0f; rotation += step) //-V1034
	{
		vec3_t pos(radius * cos(rotation) + origin.x, radius * sin(rotation) + origin.y, origin.z);

		if (render::WorldToScreen2(pos, screenPos))
		{
			if (!prevScreenPos.IsZero() && prevScreenPos.IsValid() && screenPos.IsValid() && prevScreenPos != screenPos)
			{
				line(prevScreenPos.x, prevScreenPos.y, screenPos.x, screenPos.y, color);
				triangle(vec2_t(screen.x, screen.y), vec2_t(screenPos.x, screenPos.y), vec2_t(prevScreenPos.x, prevScreenPos.y), Color(color.r(), color.g(), color.b(), 50));
			}

			prevScreenPos = screenPos;
		}
	}
}

Vertex render::RotateVertex( const vec2_t& p, const Vertex& v, float angle ) {
	// convert theta angle to sine and cosine representations.
	float c = std::cos( math::deg_to_rad( angle ) );
	float s = std::sin( math::deg_to_rad( angle ) );

	return {
        p.x + ( v.m_pos.x - p.x ) * c - ( v.m_pos.y - p.y ) * s, 
        p.y + ( v.m_pos.x - p.x ) * s + ( v.m_pos.y - p.y ) * c 
    };
}

void render::Font::string( int x, int y, Color color, const std::string& text,StringFlags_t flags /*= render::ALIGN_LEFT */ ) {
	wstring( x, y, color, util::MultiByteToWide( text ), flags );
}

void render::Font::string( int x, int y, Color color, const std::stringstream& text, StringFlags_t flags /*= render::ALIGN_LEFT */ ) {
	wstring( x, y, color, util::MultiByteToWide( text.str( ) ), flags );
}

void render::Font::wstring( int x, int y, Color color, const std::wstring& text, StringFlags_t flags /*= render::ALIGN_LEFT */ ) {
	int w, h;

	g_csgo.m_surface->GetTextSize( m_handle, text.c_str( ), w, h );
	g_csgo.m_surface->DrawSetTextFont( m_handle );
	g_csgo.m_surface->DrawSetTextColor( color );

	if( flags & ALIGN_RIGHT )
		x -= w;
	if( flags & render::ALIGN_CENTER )
		x -= w / 2;

	g_csgo.m_surface->DrawSetTextPos( x, y );
	g_csgo.m_surface->DrawPrintText( text.c_str( ), ( int )text.size( ) );
}

void render::Font::semi_filled_text_noshadow(int x, int y, Color color, const std::string& text, StringFlags_t flags, float factor)
{

	auto indicator_size = wsize(util::MultiByteToWide(text));
	auto position = vec2_t(x, y);

	//wstring(x, y, Color(30, 30, 30, 200), util::MultiByteToWide(text), flags);
	*(bool*)((DWORD)g_csgo.m_surface + 0x280) = true;
	int x1, y1, x2, y2;
	g_csgo.m_surface->get_drawing_area(x1, y1, x2, y2);
	g_csgo.m_surface->limit_drawing_area(position.x, position.y, int(indicator_size.m_width * factor), (int)indicator_size.m_height);


	wstring(x, y, color, util::MultiByteToWide(text), flags);

	g_csgo.m_surface->limit_drawing_area(x1, y1, x2, y2);
	*(bool*)((DWORD)g_csgo.m_surface - +0x280) = false;

}

render::FontSize_t render::Font::size( const std::string& text ) {
	return wsize( util::MultiByteToWide( text ) );
}

render::FontSize_t render::Font::wsize( const std::wstring& text ) {
	FontSize_t res;
	g_csgo.m_surface->GetTextSize( m_handle, text.data( ), res.m_width, res.m_height );
	return res;
}

int render::r_add_glow_box(vec3_t origin, ang_t ang_orientation, vec3_t mins, vec3_t maxs, Color clr, float lifetime) {
	static auto original = g_csgo.GlowBox;
	return original(g_csgo.m_glow, origin, ang_orientation, mins, maxs, clr, lifetime);
}

void render::rotating_circle_gradient(vec3_t pos, Color color, int point_count, float radius, bool fade, float rot_start, float fade_start, float fade_length) {
	float step = math::pi * 2.0f / point_count;
	std::vector<vec3_t> points3d;

	int alpha = 255;
	int fade_start_point = 0;
	int fade_end_point = 0;
	int fade_step = 0;

	if (fade) {
		fade_start_point = static_cast<int>(point_count * fade_start);
		fade_end_point = fade_start_point + static_cast<int>(point_count * fade_length);
		fade_step = 255 / (fade_end_point - fade_start_point);
	}

	Color outer_color = Color(0, 0, 0, 255);
	Color inner_color = color;

	for (int i = 0; i < point_count; i++) {
		if (fade && i > fade_end_point)
			break;

		float theta = (i * step) - (math::pi * 2.f * rot_start);

		vec3_t world_start(radius * cosf(theta) + pos.x, radius * sinf(theta) + pos.y, pos.z);
		vec3_t world_end(radius * cosf(theta + step) + pos.x, radius * sinf(theta + step) + pos.y, pos.z);

		vec2_t start, end;
		if (!WorldToScreen(world_start, start) || !WorldToScreen(world_end, end))
			return;

		if (fade && i >= fade_start_point) {
			alpha -= fade_step;

			if (alpha < 0)
				alpha = 0;
		}

		outer_color.malpha(alpha);
		inner_color.malpha(alpha);

		ang_t trajectory_angles;
		vec3_t ang_orientation = world_end - world_start;
		math::VectorAngles(ang_orientation, trajectory_angles);

		auto thickness = 0.2f;

		vec3_t mins = vec3_t(0.f, -thickness, -thickness);
		vec3_t maxs = vec3_t(ang_orientation.length(), thickness, thickness);

		r_add_glow_box(world_start, trajectory_angles, mins, maxs, inner_color, 3.f * g_csgo.m_globals->m_frametime);
	}
}


vec2_t render::GetScreenSize() {
	return vec2_t(g_cl.m_width, g_cl.m_height);
}