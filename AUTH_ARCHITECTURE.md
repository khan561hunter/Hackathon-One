# Authentication Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                         User Browser                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    Docusaurus App                         │  │
│  │                                                            │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │  │
│  │  │   Public     │  │  Protected   │  │    Auth      │   │  │
│  │  │   Pages      │  │   Routes     │  │   Modals     │   │  │
│  │  │              │  │              │  │              │   │  │
│  │  │  • Intro     │  │ ProtectedR..│  │ • LoginModal │   │  │
│  │  │  • ROS Intro │  │ wrapper      │  │ • SignupM... │   │  │
│  │  └──────────────┘  └──────────────┘  └──────────────┘   │  │
│  │                                                            │  │
│  │  ┌────────────────────────────────────────────────────┐  │  │
│  │  │          AuthContext (State Management)            │  │  │
│  │  │  • Session state                                   │  │  │
│  │  │  • Redirect management (intendedRoute)            │  │  │
│  │  │  • Progress tracking (userProgress)               │  │  │
│  │  └────────────────────────────────────────────────────┘  │  │
│  │                          ▲                                │  │
│  │                          │                                │  │
│  │  ┌───────────────────────┴────────────────────────────┐  │  │
│  │  │       Better Auth Client (auth-client.js)          │  │  │
│  │  │  • signIn.email()                                  │  │  │
│  │  │  • signUp.email()                                  │  │  │
│  │  │  • signIn.social()                                 │  │  │
│  │  │  • signOut()                                       │  │  │
│  │  └────────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────────┘  │
│                          │                                      │
│  ┌──────────────────────┴──────────────────────────────────┐  │
│  │              localStorage                                │  │
│  │  • auth_intended_route                                  │  │
│  │  • user_progress                                        │  │
│  └─────────────────────────────────────────────────────────┘  │
└───────────────────────────┬──────────────────────────────────┘
                            │
                            │ HTTPS (credentials: include)
                            │
┌───────────────────────────┴──────────────────────────────────┐
│                   Auth Server (Express + Better Auth)         │
│                                                                │
│  Railway: https://serene-mercy-production-5114.up.railway.app │
│  Local: http://localhost:3001                                 │
│                                                                │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │                Better Auth Endpoints                     │ │
│  │  • POST /api/auth/sign-in/email                         │ │
│  │  • POST /api/auth/sign-up/email                         │ │
│  │  • POST /api/auth/sign-in/social                        │ │
│  │  • POST /api/auth/sign-out                              │ │
│  │  • GET  /api/auth/session                               │ │
│  └─────────────────────────────────────────────────────────┘ │
│                          │                                    │
│                          ▼                                    │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │              PostgreSQL Database (Neon)                 │ │
│  │  Tables:                                                │ │
│  │  • user                                                 │ │
│  │  • session                                              │ │
│  │  • account                                              │ │
│  │  • verification                                         │ │
│  └─────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────┘
```

## Authentication Flow

### 1. Unauthenticated User Visits Protected Page

```
User navigates to /docs/advanced-topics
              ↓
    ┌─────────────────┐
    │ ProtectedRoute  │
    │   Component     │
    └────────┬────────┘
             │
             ├──→ Check: isAuthenticated?
             │
             ├──→ NO → Store current URL in localStorage
             │         key: "auth_intended_route"
             │         value: "/docs/advanced-topics"
             │
             └──→ Show SignupModal with context message
                  "Sign up to unlock Advanced Topics and
                   track your learning progress"
```

### 2. User Signs Up

```
User fills signup form (name, email, password)
              ↓
    ┌─────────────────┐
    │  SignupModal    │
    └────────┬────────┘
             │
             ├──→ Call: signUp.email({ name, email, password })
             │
             ├──→ Better Auth Client sends POST request
             │    to /api/auth/sign-up/email
             │
             ↓
    ┌─────────────────┐
    │  Auth Server    │
    └────────┬────────┘
             │
             ├──→ Hash password
             ├──→ Create user in database
             ├──→ Create session
             ├──→ Set httpOnly cookie
             │
             └──→ Return success + session
                  ↓
    ┌─────────────────┐
    │  SignupModal    │
    └────────┬────────┘
             │
             ├──→ Get stored route from localStorage
             │    ("auth_intended_route")
             │
             ├──→ Clear stored route
             │
             └──→ window.location.href = stored route
                  User redirected to /docs/advanced-topics
```

### 3. Authenticated User Sees Content

```
Page loads with user authenticated
              ↓
    ┌─────────────────┐
    │ ProtectedRoute  │
    └────────┬────────┘
             │
             ├──→ Check: isAuthenticated?
             │
             └──→ YES → Render children
                  ↓
         ┌────────────────┐
         │ WelcomeBanner  │
         └────────┬───────┘
                  │
                  ├──→ Show personalized greeting
                  │    "Good morning, [FirstName]!"
                  │
                  └──→ Show last visited page
                       "Continue where you left off: Introduction"
                  ↓
         ┌────────────────┐
         │ Page Content   │
         └────────┬───────┘
                  │
                  └──→ trackPageVisit() called
                       Update localStorage with:
                       - lastVisited: { pageId, pageTitle, timestamp }
                       - visitedPages: { [id]: { visits, lastVisit } }
```

## Component Relationships

```
Root.js (Theme wrapper)
    │
    ├──→ AuthProvider (wraps entire app)
    │    │
    │    └──→ Provides: user, session, isAuthenticated,
    │              storeIntendedRoute, getAndClearIntendedRoute,
    │              userProgress, trackPageVisit
    │
    └──→ App Content
         │
         ├──→ Navbar
         │    │
         │    ├──→ AuthButton (shows login/signup buttons)
         │    │    │
         │    │    └──→ Opens LoginModal or SignupModal
         │    │
         │    └──→ UserMenu (when authenticated)
         │         │
         │         └──→ Shows user info + sign out
         │
         └──→ Doc Pages
              │
              ├──→ Public Pages (no wrapper)
              │    │
              │    └──→ Rendered normally
              │
              └──→ Protected Pages (wrapped)
                   │
                   └──→ ProtectedRoute
                        │
                        ├──→ isAuthenticated?
                        │    │
                        │    ├──→ YES → Render content
                        │    │         │
                        │    │         └──→ WelcomeBanner
                        │    │              Page Content
                        │    │
                        │    └──→ NO → Show lock icon + message
                        │              Open SignupModal
                        │
                        └──→ LoginModal / SignupModal
                             │
                             └──→ On success:
                                  - Get intended route
                                  - Redirect to that route
```

## State Management Flow

```
┌────────────────────────────────────────────────────────────┐
│                     AuthContext State                       │
├────────────────────────────────────────────────────────────┤
│                                                              │
│  Session State (from Better Auth):                          │
│  ┌───────────────────────────────────────────────────┐     │
│  │ user: { id, name, email, image }                  │     │
│  │ session: { token, expiresAt }                     │     │
│  │ isAuthenticated: boolean                          │     │
│  │ isLoading: boolean                                │     │
│  └───────────────────────────────────────────────────┘     │
│                                                              │
│  Redirect State (localStorage):                             │
│  ┌───────────────────────────────────────────────────┐     │
│  │ intendedRoute: string | null                      │     │
│  │   ↓                                                │     │
│  │   Stored before auth: "/docs/advanced-topics"     │     │
│  │   Retrieved after auth: redirect user to it       │     │
│  │   Cleared after redirect                          │     │
│  └───────────────────────────────────────────────────┘     │
│                                                              │
│  Progress State (localStorage):                             │
│  ┌───────────────────────────────────────────────────┐     │
│  │ userProgress: {                                   │     │
│  │   lastVisited: {                                  │     │
│  │     pageId: "advanced-topics",                    │     │
│  │     pageTitle: "Advanced Topics",                 │     │
│  │     timestamp: "2025-12-16T10:30:00Z"             │     │
│  │   },                                               │     │
│  │   visitedPages: {                                 │     │
│  │     "intro": { visits: 3, lastVisit: "..." },    │     │
│  │     "advanced-topics": { visits: 1, ... }         │     │
│  │   }                                                │     │
│  │ }                                                  │     │
│  └───────────────────────────────────────────────────┘     │
└────────────────────────────────────────────────────────────┘
```

## Redirect Logic Detail

```
BEFORE AUTH:
┌─────────────────────────────────────────────────────────┐
│ User at: /docs/intro (public, authenticated)            │
│ Clicks link: /docs/advanced-topics (protected)          │
│                                                          │
│ ProtectedRoute detects: !isAuthenticated                │
│   ↓                                                      │
│ storeIntendedRoute("/docs/advanced-topics")             │
│   ↓                                                      │
│ localStorage.setItem(                                    │
│   "auth_intended_route",                                │
│   "/docs/advanced-topics"                               │
│ )                                                        │
│   ↓                                                      │
│ Show SignupModal                                         │
└─────────────────────────────────────────────────────────┘

DURING AUTH:
┌─────────────────────────────────────────────────────────┐
│ User fills form and clicks "Sign Up"                    │
│   ↓                                                      │
│ signUp.email({ email, password, name })                 │
│   ↓                                                      │
│ Auth server creates user + session                      │
│   ↓                                                      │
│ Response: { success: true }                             │
└─────────────────────────────────────────────────────────┘

AFTER AUTH:
┌─────────────────────────────────────────────────────────┐
│ SignupModal receives success                            │
│   ↓                                                      │
│ const intendedRoute =                                    │
│   localStorage.getItem("auth_intended_route")           │
│   // Returns: "/docs/advanced-topics"                   │
│   ↓                                                      │
│ localStorage.removeItem("auth_intended_route")          │
│   ↓                                                      │
│ if (intendedRoute) {                                     │
│   window.location.href = intendedRoute                  │
│ } else {                                                 │
│   window.location.reload()                              │
│ }                                                        │
│   ↓                                                      │
│ Browser navigates to: /docs/advanced-topics             │
│   ↓                                                      │
│ ProtectedRoute checks: isAuthenticated? YES             │
│   ↓                                                      │
│ Render content + WelcomeBanner                          │
└─────────────────────────────────────────────────────────┘
```

## Progress Tracking Logic

```
User views /docs/advanced-topics (while authenticated)
              ↓
    ┌─────────────────┐
    │   useEffect     │
    │  in component   │
    └────────┬────────┘
             │
             └──→ Call: trackPageVisit("advanced-topics", "Advanced Topics")
                  ↓
              ┌───────────────────┐
              │  trackPageVisit   │
              │   (AuthContext)   │
              └─────────┬─────────┘
                        │
                        ├──→ Create/update progress object:
                        │    {
                        │      lastVisited: {
                        │        pageId: "advanced-topics",
                        │        pageTitle: "Advanced Topics",
                        │        timestamp: new Date().toISOString()
                        │      },
                        │      visitedPages: {
                        │        "advanced-topics": {
                        │          title: "Advanced Topics",
                        │          lastVisit: now,
                        │          visits: (previous + 1)
                        │        }
                        │      }
                        │    }
                        │
                        └──→ Save to localStorage:
                             localStorage.setItem(
                               "user_progress",
                               JSON.stringify(progress)
                             )
                             ↓
                  WelcomeBanner reads this data
                  Shows: "Continue where you left off"
```

## Security Considerations

```
┌────────────────────────────────────────────────────────────┐
│                    Security Measures                        │
├────────────────────────────────────────────────────────────┤
│                                                              │
│ 1. Cookies (Better Auth):                                   │
│    • httpOnly: true (prevents XSS access)                   │
│    • secure: true (production HTTPS only)                   │
│    • sameSite: 'lax' (CSRF protection)                      │
│                                                              │
│ 2. CORS Configuration:                                      │
│    • credentials: 'include' (send cookies)                  │
│    • Whitelisted origins only                               │
│                                                              │
│ 3. Password Security:                                       │
│    • Min 8 characters                                       │
│    • Bcrypt hashing (Better Auth handles)                   │
│    • No plaintext storage                                   │
│                                                              │
│ 4. Client-Side Storage:                                     │
│    • localStorage for non-sensitive data only               │
│    • No auth tokens in localStorage                         │
│    • Progress data is not security-critical                 │
│                                                              │
│ 5. Route Protection:                                        │
│    • Client-side (UX only)                                  │
│    • Add server-side checks if serving sensitive content    │
│                                                              │
└────────────────────────────────────────────────────────────┘
```

## Deployment Architecture

```
Production Environment:

┌──────────────────────────────────────────────────────────┐
│                    Vercel / Netlify                       │
│              (Docusaurus Static Site)                     │
│                                                            │
│  https://yourdomain.com                                   │
│    │                                                       │
│    └─→ Static HTML/JS/CSS                                 │
│        │                                                   │
│        └─→ auth-client.js configured to:                  │
│            https://serene-mercy-production-5114...        │
└────────────────────────┬─────────────────────────────────┘
                         │
                         │ API Requests
                         │ (with cookies)
                         ↓
┌──────────────────────────────────────────────────────────┐
│                       Railway                             │
│                  (Auth Server)                            │
│                                                            │
│  https://serene-mercy-production-5114.up.railway.app     │
│    │                                                       │
│    ├─→ Express.js                                         │
│    ├─→ Better Auth                                        │
│    └─→ CORS configured for your domain                   │
└────────────────────────┬─────────────────────────────────┘
                         │
                         │ Database Queries
                         ↓
┌──────────────────────────────────────────────────────────┐
│                      Neon                                 │
│                 (PostgreSQL)                              │
│                                                            │
│  User data, sessions, accounts                           │
└──────────────────────────────────────────────────────────┘
```

---

This architecture provides:
- ✅ Secure authentication with httpOnly cookies
- ✅ Seamless UX with automatic redirects
- ✅ Personalized user experience with progress tracking
- ✅ Scalable deployment (static site + serverless)
- ✅ Easy to maintain and extend
