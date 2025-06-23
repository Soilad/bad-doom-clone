#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 400

#define SW2 (SCREEN_WIDTH/2.0f)
#define SH2 (SCREEN_HEIGHT/2.0f)

#define DEG2RAD (M_PI/180.0f)
#define EPSILON 0.000001f

#define FOV (90.0f*DEG2RAD)

#define MAX(a, b) (a) > (b) ? (a) : (b)
#define MIN(a, b) (a) < (b) ? (a) : (b)
#define CLAMP(a, b, c) MAX(b, MIN(c, a))

#define VEC2ZERO (pol_Vec2){0}

#define SECTOR_FLAG 0x80000000

typedef struct pol
{
	float x, y;
} pol_Vec2;

typedef struct
{
	Uint8 r, g, b, a;
} pol_Color;


typedef struct
{
	pol_Vec2 pos;
	float height;
	float view_angle;
} PlayerCam;

typedef struct
{
	int v1, v2;
} LineSegment;

typedef struct
{
	LineSegment *line_seg;
	float floor_height, ceiling_height;
	SDL_Surface *tex;
} DrawSegment;

typedef struct
{
	int x;
	int tex_x;
	int y1, y2;
	float v_start, v_end;
	float sy1, sy2;
} DrawColumn;

typedef struct
{
	int start_row;
	int end_row;
	int x;
	float normalized_x;
	float view_plane_height;
	PlayerCam *player_cam;
} DrawPlaneColumn;

typedef struct
{
	int num_segments;
	LineSegment *line_segs;
} Sector;

// Either a subsector or node
typedef struct Node
{
	LineSegment splitter;
	int left;
	int right;
	SDL_bool left_is_sector;
	SDL_bool right_is_sector;
} Node;

typedef struct
{
	PlayerCam player_cam;
	pol_Vec2 *vertices;
	size_t num_vertices;
	Node *nodes;
	size_t num_nodes;
	Sector *sectors;
	size_t num_sectors;
} GameState;

typedef enum
{
	POL_KEY_FORWARD,
	POL_KEY_BACK,
	POL_KEY_STRAFE_RIGHT,
	POL_KEY_STRAFE_LEFT,
	POL_KEY_TURN_LEFT,
	POL_KEY_TURN_RIGHT,
	POL_KEY_ZOOM,
	POL_KEY_ASCEND,
	POL_KEY_DESCEND,
	POL_KEY_COUNT
} pol_Key;

SDL_bool global_is_running = SDL_TRUE;
float global_yScale = (float)SCREEN_WIDTH/SCREEN_HEIGHT;
float global_focal_length;

inline float vec2_dot_product(pol_Vec2 v1, pol_Vec2 v2)
{
	return v1.x*v2.x + v1.y*v2.y;
}

inline float vec2_cross_product(pol_Vec2 v1, pol_Vec2 v2)
{
	return v1.x * v2.y - v1.y*v2.x;
}


pol_Vec2 vec2_rotate(pol_Vec2 v, float angle)
{
    pol_Vec2 result;

    float cosres = SDL_cosf(angle);
    float sinres = SDL_sinf(angle);

    result.x = v.x*cosres - v.y*sinres;
    result.y = v.x*sinres + v.y*cosres;

    return result;
}

inline float vec2_angle(pol_Vec2 v1, pol_Vec2 v2)
{
    float result;

    float dot = v1.x*v2.x + v1.y*v2.y;
    float det = v1.x*v2.y - v1.y*v2.x;

    result = SDL_atan2f(det, dot);

    return result;
}

inline float vec2_len(pol_Vec2 v1)
{
	return SDL_sqrtf(v1.x * v1.x + v1.y * v1.y);
}

pol_Vec2 line_intersect(pol_Vec2 v1, pol_Vec2 v2, pol_Vec2 v3, pol_Vec2 v4)
{
	pol_Vec2 v;

	float det = (v1.x - v2.x)*(v3.y - v4.y) - (v1.y - v2.y)*(v3.x - v4.x);
	if (SDL_fabsf(det) < EPSILON)
		return (pol_Vec2){NAN, NAN};

	v.x = ((v1.x*v2.y - v1.y*v2.x)*(v3.x - v4.x) - (v1.x - v2.x)*(v3.x*v4.y - v3.y*v4.x))/det;
	v.y = ((v1.x*v2.y - v1.y*v2.x)*(v3.y - v4.y) - (v1.y - v2.y)*(v3.x*v4.y - v3.y*v4.x))/det;

	return v;
}

pol_Vec2 line_segment_intersect(pol_Vec2 v1, pol_Vec2 v2, pol_Vec2 v3, pol_Vec2 v4)
{
    pol_Vec2 v = {NAN, NAN};

    float det = (v1.x - v2.x)*(v3.y - v4.y) - (v1.y - v2.y)*(v3.x - v4.x);
    if (SDL_fabsf(det) < EPSILON)
        return v;

    float t_num = (v1.x - v3.x)*(v3.y - v4.y) - (v1.y - v3.y)*(v3.x - v4.x);
    float u_num = (v1.x - v2.x)*(v1.y - v3.y) - (v1.y - v2.y)*(v1.x - v3.x);

    float t = t_num / det;
    float u = -u_num / det;

    if (t < 0.0f || t > 1.0f || u < 0.0f || u > 1.0f)
        return v;

    v.x = v1.x + t*(v2.x - v1.x);
    v.y = v1.y + t*(v2.y - v1.y);

    return v;
}

inline pol_Vec2 vec2_subtract(pol_Vec2 v1, pol_Vec2 v2)
{
	pol_Vec2 result;
	result.x = v1.x - v2.x;
	result.y = v1.y - v2.y;

	return result;
}

inline pol_Vec2 vec2_add(pol_Vec2 v1, pol_Vec2 v2)
{
	pol_Vec2 result;
	result.x = v1.x + v2.x;
	result.y = v1.y + v2.y;

	return result;
}

inline int point_on_side(pol_Vec2 v1, pol_Vec2 v2, pol_Vec2 p)
{
	float cross = vec2_cross_product(vec2_subtract(v2, v1), vec2_subtract(p, v1));

	// On the line
	if (SDL_fabsf(cross) < EPSILON)
		return 0;

	if (cross > 0)
		return -1;

	return 1;
}

pol_Vec2 view_to_world(pol_Vec2 v, PlayerCam *player_cam)
{
	v = vec2_rotate(v, player_cam->view_angle - 90.0f*DEG2RAD);
	v = vec2_add(v, player_cam->pos);
	return v;
}

pol_Vec2 world_to_view(pol_Vec2 v, PlayerCam *player_cam)
{
	v = vec2_subtract(v, player_cam->pos);
	v = vec2_rotate(v, -player_cam->view_angle + 90.0f*DEG2RAD);
	return v;
}

void draw_column(pol_Color *pixels, DrawColumn *column, SDL_Surface *tex)
{
	int y1 = column->y1;
	int y2 = column->y2;
	float sy1 = column->sy1;
	float sy2 = column->sy2;
	float v_start = column->v_start;
	float v_end = column->v_end;
	int x = column->x;
	int tex_x = column->tex_x;

	float deltay = v_end - v_start;
	float deltax = sy2 - sy1;
	float slope = deltay/deltax;

	float v = slope*(y1 + 0.5f - sy1) + v_start;

	for (int y = y1; y <= y2; y++)
	{
		// Due to fp errors, v can be slightly below 0.
		if (v < 0)
			v = 0;

		int tex_y = (v - SDL_floorf(v)) * tex->h;

		pol_Color *tex_pixels = tex->pixels;

		pol_Color c = tex_pixels[tex_x + tex_y * tex->w];
		pixels[x + y*SCREEN_WIDTH] = c;

		v += slope;
	}
}

void draw_plane_column(pol_Color *pixels, SDL_Surface *tex, DrawPlaneColumn *column)
{
	int start_row = column->start_row;
	int end_row = column->end_row;
	float view_plane_height = column->view_plane_height;
	float normalized_x = column->normalized_x;
	PlayerCam *player_cam = column->player_cam;
	int x = column->x;

	for (int y = start_row; y <= end_row; y++)
	{
		float normalized_y = (SH2 - y + 0.5f) / (SH2 * global_yScale);

		pol_Vec2 floor;
		floor.y = view_plane_height * global_focal_length / normalized_y;
		floor.x = normalized_x / global_focal_length * floor.y;

		floor = view_to_world(floor, player_cam);

		float tiley = floor.y / 32.0f;
		float tilex = floor.x / 32.0f;

		int tex_y = (tiley - SDL_floorf(tiley)) * tex->h;
		int tex_x = (tilex - SDL_floorf(tilex)) * tex->w;
		tex_y = CLAMP(tex_y, 0, tex->h-1);
		tex_x = CLAMP(tex_x, 0, tex->w-1);

		if (tex_x == tex->w-1 || tex_x == 0 || tex_y == tex->h-1 || tex_y == 0)
		{
			pixels[x + y*SCREEN_WIDTH] = (pol_Color){0};
			continue;
		}

		pol_Color *tex_pixels = tex->pixels;
		pixels[x + y*SCREEN_WIDTH] = tex_pixels[tex_x + tex_y*tex->w];
	}
}

void render_line_segment(pol_Color *pixels, GameState *game, DrawSegment *draw_seg)
{
	LineSegment *line_seg = draw_seg->line_seg;
	pol_Vec2 v1 = game->vertices[line_seg->v1];
	pol_Vec2 v2 = game->vertices[line_seg->v2];
	float floor_height = draw_seg->floor_height;
	float ceiling_height = draw_seg->ceiling_height;
	SDL_Surface *tex = draw_seg->tex;

	v1 = world_to_view(v1, &game->player_cam);
	v2 = world_to_view(v2, &game->player_cam);
	float view_floor_height = floor_height - game->player_cam.height;
	float view_ceiling_height = ceiling_height - game->player_cam.height;

	if (v1.y <= 0 && v2.y <= 0)
		return;

	// Backface culling
	// See: https://gamemath.com/book/graphics.html#backface_culling
	if (point_on_side(v1, v2, (pol_Vec2){0}) != 1)
		return;

	// Vectors for view edges
	pol_Vec2 clipping_v1 = vec2_rotate((pol_Vec2){0, 10000.0f},  FOV/2);
	pol_Vec2 clipping_v2 = vec2_rotate((pol_Vec2){0, 10000.0f}, -FOV/2);

	// Clipped vectors
	pol_Vec2 clipped_v1 = line_segment_intersect((pol_Vec2){0}, clipping_v1, v1, v2);
	pol_Vec2 clipped_v2 = line_segment_intersect((pol_Vec2){0}, clipping_v2, v1, v2);

	float len = vec2_len((pol_Vec2){v2.x - v1.x, v2.y - v1.y});

	float u_start = 0.0f;
	float u_end = len / tex->w;
	float v_start = 0.0f;
	float v_end = (view_ceiling_height - view_floor_height) / tex->h;

	// Clip v1 and v2 if intersection found
	if (!isnanf(clipped_v1.x))
	{
		float cliplen = vec2_len(vec2_subtract(clipped_v1, v1));
		u_start = cliplen / tex->w;
		v1 = clipped_v1;
	}
	if (!isnanf(clipped_v2.x))
	{
		float cliplen = vec2_len(vec2_subtract(clipped_v2, v2));
		u_end -= cliplen / tex->w;
		v2 = clipped_v2;
	}

	float tex_scale = 1.0f;
	u_start *= tex_scale;
	u_end *= tex_scale;
	v_start *= tex_scale;
	v_end *= tex_scale;

	// Cull walls outside of view
	const pol_Vec2 UP = {0, 1.0f};
	float angle1 = vec2_angle(UP, v1);
	float angle2 = vec2_angle(UP,v2);
	if (angle1 < -FOV/2 || angle2 > FOV/2)
		return;

	// Distance from the optical point to the projection plane

	// "NDC" coordinates
	float normalized_x1 = v1.x / v1.y * global_focal_length;
	float normalized_y1a = view_ceiling_height / v1.y * global_focal_length;
	float normalized_y1b = view_floor_height / v1.y * global_focal_length;

	float normalized_x2 = v2.x * global_focal_length / v2.y;
	float normalized_y2a = view_ceiling_height * global_focal_length / v2.y;
	float normalized_y2b = view_floor_height * global_focal_length / v2.y;

	// Screen coordinates
	float screen_x1 = SW2 + normalized_x1 * SW2;
	float screen_x2 = SW2 + normalized_x2 * SW2;
	float screen_y1a = SH2 - normalized_y1a * SH2 * global_yScale;
	float screen_y1b = SH2 - normalized_y1b * SH2 * global_yScale;
	float screen_y2a = SH2 - normalized_y2a * SH2 * global_yScale;
	float screen_y2b = SH2 - normalized_y2b * SH2 * global_yScale;

	float deltax = screen_x2 - screen_x1;
	if (SDL_fabsf(deltax) < EPSILON)
		return;
	float slope1 = (screen_y2a - screen_y1a) / deltax;
	float slope2 = (screen_y2b - screen_y1b) / deltax;

	int start_col = screen_x1 + 0.5f;
	int end_col = screen_x2 - 0.5f;
	int width = end_col - start_col + 1;

	float interpolated_screen_y1 = screen_y1a;
	float interpolated_screen_y2 = screen_y1b;

	for (int x = start_col; x <= end_col; x++)
	{
		// Start and end rows
		int y1 = interpolated_screen_y1 + 0.5f;
		int y2 = interpolated_screen_y2 - 0.5f;

		if (y1 >= SCREEN_HEIGHT || y2 < 0 || y2 < y1)
		{
			interpolated_screen_y1 += slope1;
			interpolated_screen_y2 += slope2;
			continue;
		}

		y1 = CLAMP(y1, 0, SCREEN_HEIGHT-1);
		y2 = CLAMP(y2, 0, SCREEN_HEIGHT-1);

		float tx = (x + 0.5f - screen_x1)/width;
		float u = ((1.0f - tx)*u_start/v1.y + tx*u_end/v2.y) /
			  ((1.0f - tx)*1/v1.y + tx*1/v2.y);
		int tex_x = (u - SDL_floorf(u)) * tex->w;

		DrawColumn column = {
			.x = x,
			.tex_x = tex_x,
			.y1 = y1,
			.y2 = y2,
			.sy1 = interpolated_screen_y1,
			.sy2 = interpolated_screen_y2,
			.v_start = v_start,
			.v_end = v_end
		};

		draw_column(pixels, &column, tex);

		float normalized_x = (x + 0.5f - SW2) / SW2;

		int floory1 = interpolated_screen_y2 + 0.5f;
		DrawPlaneColumn plane_column = {
			.x = x,
			.normalized_x = normalized_x,
			.start_row = floory1,
			.end_row = SCREEN_HEIGHT-1,
			.view_plane_height = view_floor_height,
			.player_cam = &game->player_cam
		};

		if (view_floor_height < 0)
			draw_plane_column(pixels, tex, &plane_column);

		int ceilingy2 = interpolated_screen_y1 - 0.5f;
		DrawPlaneColumn ceiling_column = {
			.x = x,
			.normalized_x = normalized_x,
			.start_row = 0,
			.end_row = ceilingy2,
			.view_plane_height = view_ceiling_height,
			.player_cam = &game->player_cam
		};

		if (view_ceiling_height > 0)
			draw_plane_column(pixels, tex, &ceiling_column);

		interpolated_screen_y1 += slope1;
		interpolated_screen_y2 += slope2;
	}
}

// NOTE(pol): Using an array of pairs of SDL_Scancode and pol_Key might be more
// convenient and the performance difference is probably nothing.
pol_Key translate_scancode_to_pol_key(SDL_Scancode scancode)
{
	switch (scancode)
	{
		case SDL_SCANCODE_UP:
		case SDL_SCANCODE_W: return POL_KEY_FORWARD;

		case SDL_SCANCODE_S:
		case SDL_SCANCODE_DOWN: return POL_KEY_BACK;

		case SDL_SCANCODE_A: return POL_KEY_STRAFE_LEFT;

		case SDL_SCANCODE_D: return POL_KEY_STRAFE_RIGHT;

		case SDL_SCANCODE_LEFT: return POL_KEY_TURN_LEFT;
		case SDL_SCANCODE_RIGHT: return POL_KEY_TURN_RIGHT;

		case SDL_SCANCODE_T: return POL_KEY_ZOOM;

		case SDL_SCANCODE_E: return POL_KEY_ASCEND;
		case SDL_SCANCODE_Q: return POL_KEY_DESCEND;

		default: return POL_KEY_COUNT;
	}
}

void handleKeyEvent(SDL_Event *event, SDL_bool *keys)
{
	if (event->type == SDL_KEYDOWN)
	{
		if (event->key.keysym.scancode == SDL_SCANCODE_ESCAPE)
		{
			global_is_running = SDL_FALSE;
			return;
		}

		pol_Key key = translate_scancode_to_pol_key(event->key.keysym.scancode);
		if (key < POL_KEY_COUNT)
			keys[key] = SDL_TRUE;
	}
	else if (event->type == SDL_KEYUP)
	{
		pol_Key key = translate_scancode_to_pol_key(event->key.keysym.scancode);
		if (key < POL_KEY_COUNT)
			keys[key] = SDL_FALSE;
	}
}

void clear_screenbuffer(pol_Color *screen_buffer)
{
	memset(screen_buffer, 0, SCREEN_WIDTH*SCREEN_HEIGHT*sizeof(pol_Color));
}

SDL_bool is_convex(LineSegment *segments, pol_Vec2 *vertices, int num_segments)
{
	for (int i = 0; i < num_segments; i++)
	{
		pol_Vec2 v1 = vertices[segments[i].v1];
		pol_Vec2 v2 = vertices[segments[i].v2];

		for (int j = 0; j < num_segments; j++)
		{
			if (i == j)
				continue;

			pol_Vec2 v3 = vertices[segments[j].v1];
			pol_Vec2 v4 = vertices[segments[j].v2];

			int a = point_on_side(v1, v2, v3);
			int b = point_on_side(v1, v2, v4);

			// Intersection found
			if (a * b == -1)
				return SDL_FALSE;

			if (a == -1 || b == -1)
				return SDL_FALSE;
		}
	}

	return SDL_TRUE;
}

void split_segments(LineSegment *segments, int num_segs, LineSegment **outleft, LineSegment **outright, int *outnum_left, int *outnum_right, GameState *game)
{
	pol_Vec2 split_v1 = game->vertices[segments->v1];
	pol_Vec2 split_v2 = game->vertices[segments->v2];

	LineSegment *left = NULL;
	LineSegment *right = NULL;
	int num_left = 0;
	int num_right = 0;
	
	// Copy segments and put them in either the left or right list
	for (int i = 1; i < num_segs; i++)
	{
		pol_Vec2 *v1 = game->vertices + segments[i].v1;
		pol_Vec2 *v2 = game->vertices + segments[i].v2;

		int a = point_on_side(split_v1, split_v2, *v1);
		int b = point_on_side(split_v1, split_v2, *v2);

		// Intersection
		if (a * b == -1)
		{
			pol_Vec2 split_point = line_intersect(split_v1, split_v2, *v1, *v2);
			// This explodes!
			game->vertices = SDL_realloc(game->vertices, sizeof(pol_Vec2)*(game->num_vertices+1));
			game->vertices[game->num_vertices] = split_point;

			left = SDL_realloc(left, sizeof(LineSegment)*(num_left+1));
			right = SDL_realloc(right, sizeof(LineSegment)*(num_right+1));

			if (a == -1)
			{
				left[num_left++] = (LineSegment){segments[i].v1, game->num_vertices};
				right[num_right++] = (LineSegment){game->num_vertices, segments[i].v2};
			}
			else
			{
				right[num_right++] = (LineSegment){segments[i].v1, game->num_vertices};
				left[num_left++] = (LineSegment){game->num_vertices, segments[i].v2};
			}

			game->num_vertices++;

			continue;
		}

		// Right side
		if (a == 1 || b == 1)
		{
			right = SDL_realloc(right, sizeof(LineSegment)*(num_right+1));
			right[num_right++] = segments[i];
			continue;
		}
		// Left side or splitter
		else
		{
			left = SDL_realloc(left, sizeof(LineSegment)*(num_left+1));
			left[num_left++] = segments[i];
			continue;
		}
	}

	// Move splitter to right side if empty
	if (right == NULL)
	{
		right = SDL_malloc(sizeof(LineSegment)*(num_right+1));
		right[num_right++] = segments[0];
	}
	// Else in left side
	else
	{
		left = SDL_realloc(left, sizeof(LineSegment)*(num_left+1));
		left[num_left++] = segments[0];
	}

	*outleft = left;
	*outright = right;
	*outnum_left = num_left;
	*outnum_right = num_right;
}

int generate_bsp_tree(LineSegment* line_segments, int num_segs, GameState *game)
{
	Node node;
	node.splitter = line_segments[0];

	LineSegment *left;
	LineSegment *right;
	int num_left, num_right;
	split_segments(line_segments, num_segs, &left, &right, &num_left, &num_right, game);
	SDL_free(line_segments);

	int node_index = game->num_nodes;
	game->nodes = SDL_realloc(game->nodes, sizeof(Node)*(++game->num_nodes));

	if (!is_convex(left, game->vertices, num_left))
	{
		node.left_is_sector = SDL_FALSE;
		node.left = generate_bsp_tree(left, num_left, game);
	}
	else
	{
		node.left_is_sector = SDL_TRUE;
		game->sectors = realloc(game->sectors, sizeof(Sector)*(game->num_sectors+1));
		game->sectors[game->num_sectors].num_segments = num_left;
		game->sectors[game->num_sectors].line_segs = left;
		node.left = game->num_sectors | SECTOR_FLAG;
		game->num_sectors++;
	}

	if (!is_convex(right, game->vertices, num_right))
	{
		node.right_is_sector = SDL_FALSE;
		node.right = generate_bsp_tree(right, num_right, game);
	}
	else
	{
		node.right_is_sector = SDL_TRUE;
		game->sectors = realloc(game->sectors, sizeof(Sector)*(game->num_sectors+1));
		game->sectors[game->num_sectors].num_segments = num_right;
		game->sectors[game->num_sectors].line_segs = right;
		node.right = game->num_sectors | SECTOR_FLAG;
		game->num_sectors++;
	}

	game->nodes[node_index] = node;

	return node_index;
}

void init_game(GameState *game)
{
	game->player_cam = (PlayerCam){
			.height = 40.0f,
			.view_angle = 90.0f*DEG2RAD
		},

	game->num_vertices = 18,
	game->vertices = SDL_malloc(sizeof(pol_Vec2)*18);
	pol_Vec2 vertices_data[18] = {
		{-256,  256},
		{-128,  256},
		{-128,  128},
		{   0,  128},
		{ 128,  128},
		{ 128,  256},
		{ 256,  256},

		{ 256, -256},
		{ 128, -256},
		{ 128, -128},
		{   0, -128},
		{-128, -128},
		{-128, -256},
		{-256, -256},

		{ 32,    32},
		{-32,    32},
		{-32,   -32},
		{ 32,   -32}
	};
	SDL_memcpy(game->vertices, vertices_data, sizeof(pol_Vec2)*18);
}

SDL_Surface *walltex;
Sector *sectors_to_draw[128];
int num_sectors_to_draw = 0;

void render_bsp(int node, GameState *game, pol_Color *pixels)
{
	if (node & SECTOR_FLAG)
	{
		int sector_index = node&(~SECTOR_FLAG);
		Sector *s = &game->sectors[sector_index];
		sectors_to_draw[num_sectors_to_draw++] = s;


		return;
	}

	Node *n = &game->nodes[node];
	pol_Vec2 v1 = game->vertices[n->splitter.v1];
	pol_Vec2 v2 = game->vertices[n->splitter.v2];
	int side = point_on_side(v1, v2, game->player_cam.pos);

	if (side == 1)
	{
		render_bsp(n->right, game, pixels);
		render_bsp(n->left, game, pixels);
	}
	else
	{
		render_bsp(n->left, game, pixels);
		render_bsp(n->right, game, pixels);
	}
}

int main()
{
	// NOTE(pol): SDL_CreateWindow calls SDL_Init(SDL_INIT_VIDEO) implicitly
	SDL_Window *window = SDL_CreateWindow(
		"My window",
		SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		SCREEN_WIDTH, SCREEN_HEIGHT,
		0
	);

        if (!window)
	{
		fprintf(stderr, "SDL_CreateWindow failed. SDL_Error: %s\n", SDL_GetError());
		return 1;
	}

	SDL_Renderer *renderer = SDL_CreateRenderer(
		window, -1,
		SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
	);

	if (!renderer)
	{
		fprintf(stderr, "SDL_CreateRenderer failed. SDL_Error: %s\n", SDL_GetError());
		return 1;
	}

	if (!IMG_Init(IMG_INIT_PNG))
	{
		fprintf(stderr, "IMG_Init failed. SDL_Error: %s\n", SDL_GetError());
		return 1;
	}

	SDL_bool keys[POL_KEY_COUNT] = {0};

	Uint32 startTime = SDL_GetTicks();
	Uint32 elapsedTime = 0;
	int frameCount = 0;

	SDL_Texture *screen_texture = SDL_CreateTexture(
		renderer,
		SDL_PIXELFORMAT_ABGR8888,
		SDL_TEXTUREACCESS_STREAMING,
		SCREEN_WIDTH,
		SCREEN_HEIGHT
	);

	walltex = IMG_Load("greenman.png");

	pol_Color *screen_buffer;
	int pitch;

	float floor_height = 0.0f;
	float ceiling_height = 64.0f;

	GameState game = {0};
	init_game(&game);

	LineSegment line_segments_data[18] = {
		{0, 1},
		{1, 2},
		{2, 3},
		{3, 4},
		{4, 5},
		{5, 6},
		{6, 7},

		{7, 8},
		{8, 9},
		{9, 10},
		{10, 11},
		{11, 12},
		{12, 13},
		{13, 0},

		{14, 15},
		{15, 16},
		{16, 17},
		{17, 14}
	};

	int num_segments = sizeof(line_segments_data)/sizeof(line_segments_data[0]);
	LineSegment *line_segments = SDL_malloc(sizeof(LineSegment)*num_segments);
	SDL_memcpy(line_segments, line_segments_data, sizeof(LineSegment)*num_segments);

	generate_bsp_tree(line_segments, 18, &game);

	global_focal_length = 1/SDL_tanf(FOV/2);

	SDL_Event event;
	while(global_is_running)
	{
		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
				case SDL_QUIT:
				{
					global_is_running = SDL_FALSE;
				} break;

				case SDL_KEYUP:
				case SDL_KEYDOWN:
				{
					handleKeyEvent(&event, keys);
				} break;
			}
		}

		if (keys[POL_KEY_TURN_RIGHT])
			game.player_cam.view_angle -= 0.04f;
		if (keys[POL_KEY_TURN_LEFT])
			game.player_cam.view_angle += 0.04f;

		if (keys[POL_KEY_FORWARD])
		{
			game.player_cam.pos.x += SDL_cosf(game.player_cam.view_angle);
			game.player_cam.pos.y += SDL_sinf(game.player_cam.view_angle);
		}
		if (keys[POL_KEY_BACK])
		{
			game.player_cam.pos.x -= SDL_cosf(game.player_cam.view_angle);
			game.player_cam.pos.y -= SDL_sinf(game.player_cam.view_angle);
		}
		if (keys[POL_KEY_STRAFE_RIGHT])
		{
			game.player_cam.pos.x += SDL_sinf(game.player_cam.view_angle);
			game.player_cam.pos.y -= SDL_cosf(game.player_cam.view_angle);
		}
		if (keys[POL_KEY_STRAFE_LEFT])
		{
			game.player_cam.pos.x -= SDL_sinf(game.player_cam.view_angle);
			game.player_cam.pos.y += SDL_cosf(game.player_cam.view_angle);
		}
		if (keys[POL_KEY_ASCEND])
			game.player_cam.height += 0.5;
		if (keys[POL_KEY_DESCEND])
			game.player_cam.height -= 0.5;

		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
		SDL_RenderClear(renderer);

		SDL_LockTexture(screen_texture, NULL,(void*)&screen_buffer, &pitch);
		{
			num_sectors_to_draw = 0;

			clear_screenbuffer(screen_buffer);

			render_bsp(0, &game, screen_buffer);

			for (int j = num_sectors_to_draw-1; j >= 0; j--)
			{
				Sector *s = sectors_to_draw[j];

				for (int i = 0; i < s->num_segments; i++)
				{
					DrawSegment draw_seg = {
						.line_seg = s->line_segs+i,
						.floor_height = floor_height,
						.ceiling_height = ceiling_height,
						.tex = walltex
					};

					render_line_segment(screen_buffer, &game, &draw_seg);
				}
			}


		} SDL_UnlockTexture(screen_texture);

		SDL_RenderCopy(renderer, screen_texture, NULL, NULL);

		SDL_RenderPresent(renderer);
		
		frameCount += 1;
		elapsedTime += SDL_GetTicks() - startTime;
		if (elapsedTime >= 1000)
		{
			printf("FPS: %i\n", frameCount);
			elapsedTime = elapsedTime - 1000;
			frameCount = 0;
		}

		startTime = SDL_GetTicks();
	}
}
